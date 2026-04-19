#include <AccelStepper.h>
#include <Keypad.h>

// ─── Hardware pins (RAMPS 1.4) ───────────────────────────────────────────────
#define Y_STEP 54
#define Y_DIR 55
#define Y_ENABLE 38
#define X_STEP 60
#define X_DIR 61
#define X_ENABLE 56

#define SENSOR_PIN 12 // button, INPUT_PULLUP → LOW when pressed
#define POTI1 A10     // X-axis jog
#define POTI2 A12     // Y-axis jog
#define DREH_POTI A5  // laser power (used in READY mode)
#define LASER_PIN 45  // PWM

// ─── Motor constants ──────────────────────────────────────────────────────────
const float JOG_MAX_SPEED = 100.0;  // steps/sec during calibration jog
const float DRAW_MAX_SPEED = 100.0; // steps/sec during drawing moves
const float ACCELERATION = 1000.0;  // steps/s²
const int POTI_MIN = 350;           // dead-zone lower bound
const int POTI_MAX = 650;           // dead-zone upper bound

// ─── State machine ────────────────────────────────────────────────────────────
enum SystemState
{
  CALIBRATE_LEFT,
  CALIBRATE_RIGHT,
  CALIBRATE_TOP,
  CALIBRATE_BOTTOM,
  READY
};
SystemState currentState = CALIBRATE_LEFT;

// ─── Draw modes (active when READY) ──────────────────────────────────────────
enum DrawMode
{
  MODE_OUTLINE, // continuously traces canvas border
  MODE_SERIAL   // accepts M/L/P/H/S commands over serial
};
DrawMode currentDrawMode = MODE_OUTLINE;
const int DRAW_MODE_COUNT = 2;

// ─── Canvas geometry (filled after calibration) ───────────────────────────────
long X_left = 0;
long X_right = 0;
long Y_top = 0;
long Y_bot = 0;
long canvas_width_steps = 0;
long canvas_height_steps = 0;

// ─── Laser ───────────────────────────────────────────────────────────────────
const int CAL_LASER_POWER = 255; // dim pilot beam during calibration (0-255)
int currentLaserPower = 0;

// ─── Button debounce ─────────────────────────────────────────────────────────
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_MS = 200;

// ─── WASD keyboard jog (calibration phase) ───────────────────────────────────
unsigned long keyJogXUntil = 0; // millis() timestamp until X key-jog is active
unsigned long keyJogYUntil = 0;
float keyJogXSpeed = 0.0f;
float keyJogYSpeed = 0.0f;
const unsigned long KEY_JOG_MS = 20; // how long each keypress drives the motor

// ─── LED blink state ─────────────────────────────────────────────────────────
int blinkTarget = 1; // how many blinks per cycle
int blinkCount = 0;  // blinks fired so far in this cycle
bool ledState = false;
bool inPause = false;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_ON_MS = 100;
const unsigned long BLINK_OFF_MS = 100;
const unsigned long BLINK_PAUSE_MS = 700;

// ─── Serial input buffer ─────────────────────────────────────────────────────
const int SERIAL_BUF_SIZE = 32;
char serialBuf[SERIAL_BUF_SIZE];
int serialBufPos = 0;

// ─── Keypad ───────────────────────────────────────────────────────────────────
const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte rowPins[ROWS] = {23, 25, 27, 29};
byte colPins[COLS] = {31, 33, 35, 37};
Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// ─── Stepper instances ───────────────────────────────────────────────────────
AccelStepper xStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, X_STEP, X_DIR);

// ═══════════════════════════════════════════════════════════════════════════════
//  LASER
// ═══════════════════════════════════════════════════════════════════════════════

void setLaser(int power)
{
  currentLaserPower = constrain(power, 0, 255);
  analogWrite(LASER_PIN, currentLaserPower);
}

// 3 quick laser blinks to confirm a calibration point was saved
void laserConfirmBlink()
{
  for (int i = 0; i < 3; i++)
  {
    setLaser(0);
    delay(80);
    setLaser(CAL_LASER_POWER);
    delay(80);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LED BLINK  (non-blocking)
//  Blinks blinkTarget times, then pauses BLINK_PAUSE_MS, then repeats.
//  In READY state: LED is steady on.
// ═══════════════════════════════════════════════════════════════════════════════

void setBlinkTarget(int n)
{
  blinkTarget = n;
  blinkCount = 0;
  inPause = false;
  ledState = false;
  lastBlinkTime = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void updateLedBlink()
{
  if (currentState == READY)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  unsigned long now = millis();

  if (inPause)
  {
    if (now - lastBlinkTime >= BLINK_PAUSE_MS)
    {
      inPause = false;
      blinkCount = 0;
      ledState = false;
      lastBlinkTime = now;
      digitalWrite(LED_BUILTIN, LOW);
    }
    return;
  }

  unsigned long interval = ledState ? BLINK_ON_MS : BLINK_OFF_MS;
  if (now - lastBlinkTime < interval)
    return;

  lastBlinkTime = now;

  if (!ledState)
  {
    // about to turn ON — only if we haven't finished the cycle
    if (blinkCount < blinkTarget)
    {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      // all blinks done → start pause
      inPause = true;
    }
  }
  else
  {
    // turn OFF
    ledState = false;
    blinkCount++;
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BUTTON
// ═══════════════════════════════════════════════════════════════════════════════

bool checkButton()
{
  if (digitalRead(SENSOR_PIN) == LOW)
  {
    unsigned long now = millis();
    if (now - lastButtonTime > DEBOUNCE_MS)
    {
      lastButtonTime = now;
      return true;
    }
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  JOG CONTROL  (calibration phase, velocity mode)
// ═══════════════════════════════════════════════════════════════════════════════

float potiToSpeed(int raw)
{
  if (raw > POTI_MAX)
  {
    return map(raw, POTI_MAX, 1023, 0, (int)JOG_MAX_SPEED);
  }
  else if (raw < POTI_MIN)
  {
    return -map(raw, 0, POTI_MIN, (int)JOG_MAX_SPEED, 0);
  }
  return 0.0f;
}

void jogMotors()
{
  unsigned long now = millis();
  float sx = (now < keyJogXUntil) ? keyJogXSpeed : potiToSpeed(analogRead(POTI1));
  float sy = -((now < keyJogYUntil) ? keyJogYSpeed : potiToSpeed(analogRead(POTI2))); // motor mounted inverted
  xStepper.setSpeed(sx);
  yStepper.setSpeed(sy);
  xStepper.runSpeed();
  yStepper.runSpeed();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════

void restartCalibration()
{
  xStepper.stop();
  yStepper.stop();
  setLaser(CAL_LASER_POWER);
  currentState = CALIBRATE_LEFT;
  setBlinkTarget(1);
  Serial.println(F("CAL: move to LEFT limit, press button"));
}

void finalizeCalibration()
{
  // Normalize axes so left < right and top < bottom
  if (X_left > X_right)
  {
    long t = X_left;
    X_left = X_right;
    X_right = t;
  }
  if (Y_top > Y_bot)
  {
    long t = Y_top;
    Y_top = Y_bot;
    Y_bot = t;
  }

  canvas_width_steps = X_right - X_left;
  canvas_height_steps = Y_bot - Y_top;

  if (canvas_width_steps == 0 || canvas_height_steps == 0)
  {
    Serial.println(F("ERR: canvas zero-size, recalibrate"));
    restartCalibration();
    return;
  }

  // Move to origin (top-left corner) and zero the position counters
  xStepper.setMaxSpeed(DRAW_MAX_SPEED);
  yStepper.setMaxSpeed(DRAW_MAX_SPEED);
  xStepper.moveTo(X_left);
  yStepper.moveTo(Y_top);
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0)
  {
    xStepper.run();
    yStepper.run();
  }

  // Re-zero so (0,0) == top-left canvas corner
  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);
  X_right -= X_left;
  X_left = 0;
  Y_bot -= Y_top;
  Y_top = 0;

  currentState = READY;
  currentDrawMode = MODE_OUTLINE;
  Serial.print(F("READY W="));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H="));
  Serial.print(canvas_height_steps);
  Serial.println(F(" MODE: OUTLINE"));
}

void saveCalibrationPoint()
{
  long cx = xStepper.currentPosition();
  long cy = yStepper.currentPosition();

  switch (currentState)
  {
  case CALIBRATE_LEFT:
    X_left = cx;
    laserConfirmBlink();
    Serial.print(F("CAL: X_left="));
    Serial.println(X_left);
    currentState = CALIBRATE_RIGHT;
    setBlinkTarget(2);
    Serial.println(F("CAL: move to RIGHT limit, press button"));
    break;

  case CALIBRATE_RIGHT:
    X_right = cx;
    laserConfirmBlink();
    Serial.print(F("CAL: X_right="));
    Serial.println(X_right);
    currentState = CALIBRATE_TOP;
    setBlinkTarget(3);
    Serial.println(F("CAL: move to TOP limit, press button"));
    break;

  case CALIBRATE_TOP:
    Y_top = cy;
    laserConfirmBlink();
    Serial.print(F("CAL: Y_top="));
    Serial.println(Y_top);
    currentState = CALIBRATE_BOTTOM;
    setBlinkTarget(4);
    Serial.println(F("CAL: move to BOTTOM limit, press button"));
    break;

  case CALIBRATE_BOTTOM:
    Y_bot = cy;
    laserConfirmBlink();
    Serial.print(F("CAL: Y_bot="));
    Serial.println(Y_bot);
    finalizeCalibration();
    break;

  default:
    break;
  }
}

// ─── WASD serial jog (calibration only) ──────────────────────────────────────
//   A / D  →  X axis left / right
//   W / S  →  Y axis up (top) / down (bottom)
//   Space or Enter  →  save current position (same as physical button)
void handleCalibrationSerial()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    unsigned long until = millis() + KEY_JOG_MS;
    switch (c)
    {
    case 'a':
    case 'A':
      keyJogXSpeed = -JOG_MAX_SPEED;
      keyJogXUntil = until;
      break;
    case 'd':
    case 'D':
      keyJogXSpeed = JOG_MAX_SPEED;
      keyJogXUntil = until;
      break;
    case 'w':
    case 'W':
      keyJogYSpeed = -JOG_MAX_SPEED;
      keyJogYUntil = until;
      break;
    case 's':
    case 'S':
      keyJogYSpeed = JOG_MAX_SPEED;
      keyJogYUntil = until;
      break;
    case ' ':
      saveCalibrationPoint();
      break;
    }
  }
}

// ─── Keypad jog (calibration only) ───────────────────────────────────────────
//   2=up  9=down  4=left  6=right  5=save point
void handleKeypad()
{
  keypad.getKeys(); // update key states; return value ignored so HOLD is always checked
  for (int i = 0; i < LIST_MAX; i++)
  {
    if (keypad.key[i].kchar == NO_KEY)
      continue;
    KeyState s = keypad.key[i].kstate;
    if (s != PRESSED && s != HOLD)
      continue;
    char k = keypad.key[i].kchar;
    unsigned long until = millis() + KEY_JOG_MS;
    switch (k)
    {
    case '4':
      keyJogXSpeed = -JOG_MAX_SPEED;
      keyJogXUntil = until;
      break;
    case '6':
      keyJogXSpeed = JOG_MAX_SPEED;
      keyJogXUntil = until;
      break;
    case '2':
      keyJogYSpeed = -JOG_MAX_SPEED; // logical up
      keyJogYUntil = until;
      break;
    case '8':
      keyJogYSpeed = JOG_MAX_SPEED; // logical down
      keyJogYUntil = until;
      break;
    case '5':
      if (s == PRESSED)
        saveCalibrationPoint();
      break;
    case '0':
      if (s == PRESSED)
        restartCalibration();
      break;
    case '#':
      if (s == PRESSED && currentState == READY)
        setDrawMode((DrawMode)((currentDrawMode + 1) % DRAW_MODE_COUNT));
      break;
    }
  }
}

void handleCalibrationState()
{
  handleCalibrationSerial();
  handleKeypad();
  jogMotors();
  if (checkButton())
    saveCalibrationPoint();
  updateLedBlink();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODE SWITCHING
// ═══════════════════════════════════════════════════════════════════════════════

void setDrawMode(DrawMode m)
{
  xStepper.stop();
  yStepper.stop();
  setLaser(0);
  currentDrawMode = m;
  switch (m)
  {
  case MODE_OUTLINE:
    Serial.println(F("MODE: OUTLINE"));
    break;
  case MODE_SERIAL:
    Serial.println(F("MODE: SERIAL"));
    break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DRAWING PRIMITIVES  (position mode, blocking)
// ═══════════════════════════════════════════════════════════════════════════════

long normToStepsX(float n)
{
  long s = (long)(constrain(n, 0.0f, 1.0f) * canvas_width_steps);
  return constrain(s, X_left, X_left + canvas_width_steps);
}

long normToStepsY(float n)
{
  long s = (long)(constrain(n, 0.0f, 1.0f) * canvas_height_steps);
  return constrain(s, Y_top, Y_top + canvas_height_steps);
}

void waitForMotors()
{
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0)
  {
    xStepper.run();
    yStepper.run();
  }
}

void moveToCanvas(float nx, float ny)
{
  setLaser(0);
  xStepper.moveTo(normToStepsX(nx));
  yStepper.moveTo(normToStepsY(ny));
  waitForMotors();
}

void drawToCanvas(float nx, float ny, int power)
{
  setLaser(power);
  xStepper.moveTo(normToStepsX(nx));
  yStepper.moveTo(normToStepsY(ny));
  waitForMotors();
  setLaser(0);
}

void drawLine(float x0, float y0, float x1, float y1, int power)
{
  moveToCanvas(x0, y0);
  drawToCanvas(x1, y1, power);
}

// Trace the full canvas rectangle once with the laser on (called after calibration)
void traceCanvasOutline()
{
  moveToCanvas(0.0f, 0.0f); // go to top-left (laser off)
  setLaser(CAL_LASER_POWER);
  xStepper.moveTo(normToStepsX(1.0f));
  yStepper.moveTo(normToStepsY(0.0f));
  waitForMotors(); // top-right
  xStepper.moveTo(normToStepsX(1.0f));
  yStepper.moveTo(normToStepsY(1.0f));
  waitForMotors(); // bottom-right
  xStepper.moveTo(normToStepsX(0.0f));
  yStepper.moveTo(normToStepsY(1.0f));
  waitForMotors(); // bottom-left
  xStepper.moveTo(normToStepsX(0.0f));
  yStepper.moveTo(normToStepsY(0.0f));
  waitForMotors(); // close
  setLaser(0);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND INTERFACE  (READY mode only)
//
//  Commands (newline-terminated):
//    M x y   — move (laser off) to canvas coords 0-1000
//    L x y   — draw (laser on) to canvas coords 0-1000
//    P p     — set laser power 0-255 (persists for L commands)
//    H       — home to (0, 0)
//    S       — report position and canvas size
//    C       — restart calibration
//
//  All successful commands reply "OK\n".
// ═══════════════════════════════════════════════════════════════════════════════

void reportStatus()
{
  Serial.print(F("POS "));
  Serial.print(xStepper.currentPosition());
  Serial.print(' ');
  Serial.print(yStepper.currentPosition());
  Serial.print(F(" W "));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H "));
  Serial.println(canvas_height_steps);
}

void parseSerialCommand(char *cmd)
{
  int x, y, p;

  if (cmd[0] == 'M' && sscanf(cmd + 1, "%d %d", &x, &y) == 2)
  {
    moveToCanvas(x / 1000.0f, y / 1000.0f);
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'L' && sscanf(cmd + 1, "%d %d", &x, &y) == 2)
  {
    drawToCanvas(x / 1000.0f, y / 1000.0f, currentLaserPower);
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'P' && sscanf(cmd + 1, "%d", &p) == 1)
  {
    setLaser(p);
    currentLaserPower = currentLaserPower; // already set by setLaser
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'H')
  {
    moveToCanvas(0.0f, 0.0f);
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'S')
  {
    reportStatus();
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'C')
  {
    restartCalibration();
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'N')
  {
    setDrawMode((DrawMode)((currentDrawMode + 1) % DRAW_MODE_COUNT));
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("ERR"));
  }
}

void handleSerialInput()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (serialBufPos > 0)
      {
        serialBuf[serialBufPos] = '\0';
        parseSerialCommand(serialBuf);
        serialBufPos = 0;
      }
    }
    else
    {
      if (serialBufPos < SERIAL_BUF_SIZE - 1)
      {
        serialBuf[serialBufPos++] = c;
      }
      else
      {
        // Buffer overflow — discard and report error
        serialBufPos = 0;
        Serial.println(F("ERR"));
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(POTI1, INPUT);
  pinMode(POTI2, INPUT);
  pinMode(DREH_POTI, INPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(X_ENABLE, OUTPUT);
  pinMode(Y_ENABLE, OUTPUT);

  digitalWrite(X_ENABLE, LOW); // LOW = enabled on RAMPS
  digitalWrite(Y_ENABLE, LOW);

  setLaser(CAL_LASER_POWER);

  keypad.setHoldTime(150); // fire HOLD events every 150ms for smooth jog

  xStepper.setMaxSpeed(JOG_MAX_SPEED);
  xStepper.setAcceleration(ACCELERATION);
  yStepper.setMaxSpeed(JOG_MAX_SPEED);
  yStepper.setAcceleration(ACCELERATION);

  setBlinkTarget(1);
  Serial.println(F("LASER CANVAS v1.0"));
  Serial.println(F("Jog: potis or WASD keys | Space/Enter = save point"));
  Serial.println(F("CAL: move to LEFT limit, press button or Space"));
}

void loop()
{
  if (currentState != READY)
  {
    handleCalibrationState();
  }
  else
  {
    handleKeypad(); // '#' cycles modes; '0' restarts calibration
    switch (currentDrawMode)
    {
    case MODE_OUTLINE:
      traceCanvasOutline();
      break;
    case MODE_SERIAL:
      handleSerialInput();
      break;
    }
    updateLedBlink();
  }
}
