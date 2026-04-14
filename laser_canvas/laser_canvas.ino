#include <AccelStepper.h>

// ─── Hardware pins (RAMPS 1.4) ───────────────────────────────────────────────
#define X_STEP    54
#define X_DIR     55
#define X_ENABLE  38
#define Y_STEP 60
#define Y_DIR 61
#define Y_ENABLE 56

#define SENSOR_PIN  12   // button, INPUT_PULLUP → LOW when pressed
#define POTI1       A4   // X-axis jog
#define POTI2       A9   // Y-axis jog
#define DREH_POTI   A3   // laser power (used in READY mode)
#define LASER_PIN   45   // PWM

// ─── Motor constants ──────────────────────────────────────────────────────────
const float JOG_MAX_SPEED   = 100.0;  // steps/sec during calibration jog
const float DRAW_MAX_SPEED  = 100.0;  // steps/sec during drawing moves
const float ACCELERATION    = 1000.0; // steps/s²
const int   POTI_MIN        = 350;    // dead-zone lower bound
const int   POTI_MAX        = 650;    // dead-zone upper bound

// ─── State machine ────────────────────────────────────────────────────────────
enum SystemState {
  CALIBRATE_LEFT,
  CALIBRATE_RIGHT,
  CALIBRATE_TOP,
  CALIBRATE_BOTTOM,
  READY
};
SystemState currentState = CALIBRATE_LEFT;

// ─── Canvas geometry (filled after calibration) ───────────────────────────────
long X_left  = 0;
long X_right = 0;
long Y_top   = 0;
long Y_bot   = 0;
long canvas_width_steps  = 0;
long canvas_height_steps = 0;

// ─── Laser ───────────────────────────────────────────────────────────────────
int currentLaserPower = 0;

// ─── Button debounce ─────────────────────────────────────────────────────────
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_MS = 200;

// ─── WASD keyboard jog (calibration phase) ───────────────────────────────────
unsigned long keyJogXUntil = 0; // millis() timestamp until X key-jog is active
unsigned long keyJogYUntil = 0;
float keyJogXSpeed = 0.0f;
float keyJogYSpeed = 0.0f;
const unsigned long KEY_JOG_MS = 180; // how long each keypress drives the motor

// ─── LED blink state ─────────────────────────────────────────────────────────
int  blinkTarget  = 1;    // how many blinks per cycle
int  blinkCount   = 0;    // blinks fired so far in this cycle
bool ledState     = false;
bool inPause      = false;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_ON_MS  = 100;
const unsigned long BLINK_OFF_MS = 100;
const unsigned long BLINK_PAUSE_MS = 700;

// ─── Serial input buffer ─────────────────────────────────────────────────────
const int SERIAL_BUF_SIZE = 32;
char serialBuf[SERIAL_BUF_SIZE];
int  serialBufPos = 0;

// ─── Stepper instances ───────────────────────────────────────────────────────
AccelStepper xStepper(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);


// ═══════════════════════════════════════════════════════════════════════════════
//  LASER
// ═══════════════════════════════════════════════════════════════════════════════

void setLaser(int power) {
  currentLaserPower = constrain(power, 0, 255);
  analogWrite(LASER_PIN, currentLaserPower);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  LED BLINK  (non-blocking)
//  Blinks blinkTarget times, then pauses BLINK_PAUSE_MS, then repeats.
//  In READY state: LED is steady on.
// ═══════════════════════════════════════════════════════════════════════════════

void setBlinkTarget(int n) {
  blinkTarget   = n;
  blinkCount    = 0;
  inPause       = false;
  ledState      = false;
  lastBlinkTime = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void updateLedBlink() {
  if (currentState == READY) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  unsigned long now = millis();

  if (inPause) {
    if (now - lastBlinkTime >= BLINK_PAUSE_MS) {
      inPause    = false;
      blinkCount = 0;
      ledState   = false;
      lastBlinkTime = now;
      digitalWrite(LED_BUILTIN, LOW);
    }
    return;
  }

  unsigned long interval = ledState ? BLINK_ON_MS : BLINK_OFF_MS;
  if (now - lastBlinkTime < interval) return;

  lastBlinkTime = now;

  if (!ledState) {
    // about to turn ON — only if we haven't finished the cycle
    if (blinkCount < blinkTarget) {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      // all blinks done → start pause
      inPause = true;
    }
  } else {
    // turn OFF
    ledState = false;
    blinkCount++;
    digitalWrite(LED_BUILTIN, LOW);
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  BUTTON
// ═══════════════════════════════════════════════════════════════════════════════

bool checkButton() {
  if (digitalRead(SENSOR_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonTime > DEBOUNCE_MS) {
      lastButtonTime = now;
      return true;
    }
  }
  return false;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  JOG CONTROL  (calibration phase, velocity mode)
// ═══════════════════════════════════════════════════════════════════════════════

float potiToSpeed(int raw) {
  if (raw > POTI_MAX) {
    return map(raw, POTI_MAX, 1023, 0, (int)JOG_MAX_SPEED);
  } else if (raw < POTI_MIN) {
    return -map(raw, 0, POTI_MIN, (int)JOG_MAX_SPEED, 0);
  }
  return 0.0f;
}

void jogMotors() {
  unsigned long now = millis();
  float sx = (now < keyJogXUntil) ? keyJogXSpeed : potiToSpeed(analogRead(POTI1));
  float sy = (now < keyJogYUntil) ? keyJogYSpeed : potiToSpeed(analogRead(POTI2));
  xStepper.setSpeed(sx);
  yStepper.setSpeed(sy);
  xStepper.runSpeed();
  yStepper.runSpeed();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════

void restartCalibration() {
  xStepper.stop();
  yStepper.stop();
  setLaser(0);
  currentState = CALIBRATE_LEFT;
  setBlinkTarget(1);
  Serial.println(F("CAL: move to LEFT limit, press button"));
}

void finalizeCalibration() {
  // Normalize axes so left < right and top < bottom
  if (X_left > X_right) { long t = X_left; X_left = X_right; X_right = t; }
  if (Y_top  > Y_bot)   { long t = Y_top;  Y_top  = Y_bot;   Y_bot   = t; }

  canvas_width_steps  = X_right - X_left;
  canvas_height_steps = Y_bot   - Y_top;

  if (canvas_width_steps == 0 || canvas_height_steps == 0) {
    Serial.println(F("ERR: canvas zero-size, recalibrate"));
    restartCalibration();
    return;
  }

  // Move to origin (top-left corner) and zero the position counters
  xStepper.setMaxSpeed(DRAW_MAX_SPEED);
  yStepper.setMaxSpeed(DRAW_MAX_SPEED);
  xStepper.moveTo(X_left);
  yStepper.moveTo(Y_top);
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0) {
    xStepper.run();
    yStepper.run();
  }

  // Re-zero so (0,0) == top-left canvas corner
  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);
  X_right -= X_left; X_left = 0;
  Y_bot   -= Y_top;  Y_top  = 0;

  currentState = READY;
  Serial.print(F("READY W="));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H="));
  Serial.println(canvas_height_steps);
}

void saveCalibrationPoint() {
  long cx = xStepper.currentPosition();
  long cy = yStepper.currentPosition();

  switch (currentState) {
    case CALIBRATE_LEFT:
      X_left = cx;
      Serial.print(F("CAL: X_left=")); Serial.println(X_left);
      currentState = CALIBRATE_RIGHT;
      setBlinkTarget(2);
      Serial.println(F("CAL: move to RIGHT limit, press button"));
      break;

    case CALIBRATE_RIGHT:
      X_right = cx;
      Serial.print(F("CAL: X_right=")); Serial.println(X_right);
      currentState = CALIBRATE_TOP;
      setBlinkTarget(3);
      Serial.println(F("CAL: move to TOP limit, press button"));
      break;

    case CALIBRATE_TOP:
      Y_top = cy;
      Serial.print(F("CAL: Y_top=")); Serial.println(Y_top);
      currentState = CALIBRATE_BOTTOM;
      setBlinkTarget(4);
      Serial.println(F("CAL: move to BOTTOM limit, press button"));
      break;

    case CALIBRATE_BOTTOM:
      Y_bot = cy;
      Serial.print(F("CAL: Y_bot=")); Serial.println(Y_bot);
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
    case '\n':
    case '\r':
      saveCalibrationPoint();
      break;
    }
  }
}

void handleCalibrationState() {
  handleCalibrationSerial();
  jogMotors();
  if (checkButton()) saveCalibrationPoint();
  updateLedBlink();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  DRAWING PRIMITIVES  (position mode, blocking)
// ═══════════════════════════════════════════════════════════════════════════════

long normToStepsX(float n) {
  long s = (long)(constrain(n, 0.0f, 1.0f) * canvas_width_steps);
  return constrain(s, X_left, X_left + canvas_width_steps);
}

long normToStepsY(float n) {
  long s = (long)(constrain(n, 0.0f, 1.0f) * canvas_height_steps);
  return constrain(s, Y_top, Y_top + canvas_height_steps);
}

void waitForMotors() {
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0) {
    xStepper.run();
    yStepper.run();
  }
}

void moveToCanvas(float nx, float ny) {
  setLaser(0);
  xStepper.moveTo(normToStepsX(nx));
  yStepper.moveTo(normToStepsY(ny));
  waitForMotors();
}

void drawToCanvas(float nx, float ny, int power) {
  setLaser(power);
  xStepper.moveTo(normToStepsX(nx));
  yStepper.moveTo(normToStepsY(ny));
  waitForMotors();
  setLaser(0);
}

void drawLine(float x0, float y0, float x1, float y1, int power) {
  moveToCanvas(x0, y0);
  drawToCanvas(x1, y1, power);
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

void reportStatus() {
  Serial.print(F("POS "));
  Serial.print(xStepper.currentPosition());
  Serial.print(' ');
  Serial.print(yStepper.currentPosition());
  Serial.print(F(" W "));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H "));
  Serial.println(canvas_height_steps);
}

void parseSerialCommand(char* cmd) {
  int x, y, p;

  if (cmd[0] == 'M' && sscanf(cmd + 1, "%d %d", &x, &y) == 2) {
    moveToCanvas(x / 1000.0f, y / 1000.0f);
    Serial.println(F("OK"));

  } else if (cmd[0] == 'L' && sscanf(cmd + 1, "%d %d", &x, &y) == 2) {
    drawToCanvas(x / 1000.0f, y / 1000.0f, currentLaserPower);
    Serial.println(F("OK"));

  } else if (cmd[0] == 'P' && sscanf(cmd + 1, "%d", &p) == 1) {
    setLaser(p);
    currentLaserPower = currentLaserPower; // already set by setLaser
    Serial.println(F("OK"));

  } else if (cmd[0] == 'H') {
    moveToCanvas(0.0f, 0.0f);
    Serial.println(F("OK"));

  } else if (cmd[0] == 'S') {
    reportStatus();
    Serial.println(F("OK"));

  } else if (cmd[0] == 'C') {
    restartCalibration();
    Serial.println(F("OK"));

  } else {
    Serial.println(F("ERR"));
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufPos > 0) {
        serialBuf[serialBufPos] = '\0';
        parseSerialCommand(serialBuf);
        serialBufPos = 0;
      }
    } else {
      if (serialBufPos < SERIAL_BUF_SIZE - 1) {
        serialBuf[serialBufPos++] = c;
      } else {
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

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LASER_PIN,   OUTPUT);
  pinMode(POTI1,       INPUT);
  pinMode(POTI2,       INPUT);
  pinMode(DREH_POTI,   INPUT);
  pinMode(SENSOR_PIN,  INPUT_PULLUP);
  pinMode(X_ENABLE,    OUTPUT);
  pinMode(Y_ENABLE,    OUTPUT);

  digitalWrite(X_ENABLE, LOW);  // LOW = enabled on RAMPS
  digitalWrite(Y_ENABLE, LOW);

  setLaser(0);

  xStepper.setMaxSpeed(JOG_MAX_SPEED);
  xStepper.setAcceleration(ACCELERATION);
  yStepper.setMaxSpeed(JOG_MAX_SPEED);
  yStepper.setAcceleration(ACCELERATION);

  setBlinkTarget(1);
  Serial.println(F("LASER CANVAS v1.0"));
  Serial.println(F("Jog: potis or WASD keys | Space/Enter = save point"));
  Serial.println(F("CAL: move to LEFT limit, press button or Space"));
}

void loop() {
  if (currentState != READY) {
    handleCalibrationState();
  } else {
    handleSerialInput();
    updateLedBlink();
  }
}
