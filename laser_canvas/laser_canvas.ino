#include <AccelStepper.h>

// ─── Hardware pins (RAMPS 1.4) ───────────────────────────────────────────────
#define Y_STEP 54
#define Y_DIR 55
#define Y_ENABLE 38
#define X_STEP 60
#define X_DIR 61
#define X_ENABLE 56

#define BTN_JOYSTICK 42 // joystick click — saves calibration point
#define BTN_PREV 50     // previous draw mode button
#define BTN_NEXT 51     // next draw mode button
#define POTI1 A11       // X-axis jog
#define POTI2 A12     // Y-axis jog
#define LASER_PIN 45  // PWM
#define DMX_PIN 2     // RS485 direction-enable (tied DE+/RE-)
                      // RS485 data uses Serial1 TX (pin 18 on Mega)
#define DMX_ADDRESS 1 // fixture base channel (1-based)

// ─── Motor constants ──────────────────────────────────────────────────────────
const float JOG_MAX_SPEED = 100.0; // steps/sec during calibration jog
const float DRAW_MAX_SPEED = 25.0; // steps/sec during drawing moves (1/4 of jog)
const float ACCELERATION = 1000.0; // steps/s²
const int POTI_MIN = 350;          // dead-zone lower bound
const int POTI_MAX = 650;          // dead-zone upper bound

const uint32_t DN_LASER_MS = 5UL * 60UL * 1000UL; // 5 min laser drawing
const uint32_t DN_SUN_MS = 1UL * 60UL * 1000UL;   // 1 min sun DMX effect

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
  MODE_OUTLINE, // 1 — continuously traces canvas border
  MODE_CORNERS, // 2 — random small 90° corner shapes
  MODE_LETTERS, // 3 — random German characters at random positions
  MODE_CIRCLE,  // 4 — continuous circle in canvas centre
  MODE_MANUAL,  // 5 — poti jog with laser on — freehand drawing
  MODE_SERIAL,  // 6 — accepts M/L/P/H/S commands over serial
  MODE_MORSE,   // 7 — draws morse-coded words at random positions
  MODE_DAYNIGHT // 8 — 5-min laser drawing / 1-min sun DMX cycle
};
DrawMode currentDrawMode = MODE_OUTLINE;
const int DRAW_MODE_COUNT = 8;

// ─── Canvas geometry (filled after calibration) ───────────────────────────────
long X_left = 0;
long X_right = 0;
long Y_top = 0;
long Y_bot = 0;
long canvas_width_steps = 0;
long canvas_height_steps = 0;

// ─── Laser ───────────────────────────────────────────────────────────────────
int currentLaserPower = 255;
bool laserEnabled = false;

// ─── Button debounce ─────────────────────────────────────────────────────────
unsigned long lastButtonTime = 0;
unsigned long lastPrevTime = 0;
unsigned long lastNextTime = 0;
const unsigned long DEBOUNCE_MS = 200;

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

// ─── Stepper instances ───────────────────────────────────────────────────────
AccelStepper xStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, X_STEP, X_DIR);

// ═══════════════════════════════════════════════════════════════════════════════
//  STROKE FONT  (8-wide × 12-tall cell, y=0 at top)
//  Curves are approximated with ~6 waypoints per semicircle (30° steps).
//  Pairs of int8_t (x,y) | -1,-1 = pen-up | 127,127 = end of glyph
// ═══════════════════════════════════════════════════════════════════════════════
const int8_t F_A[] PROGMEM = {0, 12, 4, 0, 8, 12, -1, -1, 2, 6, 6, 6, 127, 127};
const int8_t F_B[] PROGMEM = {0, 12, 0, 0, 4, 0, 7, 2, 7, 4, 4, 6, 0, 6, 4, 6, 7, 8, 7, 10, 4, 12, 0, 12, 127, 127};
const int8_t F_C[] PROGMEM = {7, 2, 6, 1, 4, 0, 2, 1, 1, 3, 0, 6, 1, 9, 2, 11, 4, 12, 6, 11, 7, 10, 127, 127};
const int8_t F_D[] PROGMEM = {0, 12, 0, 0, 4, 0, 7, 3, 8, 6, 7, 9, 4, 12, 0, 12, 127, 127};
const int8_t F_E[] PROGMEM = {8, 0, 0, 0, 0, 12, 8, 12, -1, -1, 0, 6, 6, 6, 127, 127};
const int8_t F_F[] PROGMEM = {8, 0, 0, 0, 0, 12, -1, -1, 0, 6, 6, 6, 127, 127};
const int8_t F_G[] PROGMEM = {7, 2, 6, 1, 4, 0, 2, 1, 1, 3, 0, 6, 1, 9, 2, 11, 4, 12, 6, 11, 7, 10, 7, 6, 4, 6, 127, 127};
const int8_t F_H[] PROGMEM = {0, 0, 0, 12, -1, -1, 8, 0, 8, 12, -1, -1, 0, 6, 8, 6, 127, 127};
const int8_t F_I[] PROGMEM = {2, 0, 6, 0, -1, -1, 4, 0, 4, 12, -1, -1, 2, 12, 6, 12, 127, 127};
const int8_t F_J[] PROGMEM = {2, 0, 6, 0, -1, -1, 5, 0, 5, 9, 4, 11, 2, 12, 1, 11, 0, 9, 127, 127};
const int8_t F_K[] PROGMEM = {0, 0, 0, 12, -1, -1, 8, 0, 0, 6, 8, 12, 127, 127};
const int8_t F_L[] PROGMEM = {0, 0, 0, 12, 8, 12, 127, 127};
const int8_t F_M[] PROGMEM = {0, 12, 0, 0, 4, 6, 8, 0, 8, 12, 127, 127};
const int8_t F_N[] PROGMEM = {0, 12, 0, 0, 8, 12, 8, 0, 127, 127};
const int8_t F_O[] PROGMEM = {4, 0, 6, 1, 7, 3, 8, 6, 7, 9, 6, 11, 4, 12, 2, 11, 1, 9, 0, 6, 1, 3, 2, 1, 4, 0, 127, 127};
const int8_t F_P[] PROGMEM = {0, 12, 0, 0, 4, 0, 7, 2, 7, 4, 4, 6, 0, 6, 127, 127};
const int8_t F_Q[] PROGMEM = {4, 0, 6, 1, 7, 3, 8, 6, 7, 9, 6, 11, 4, 12, 2, 11, 1, 9, 0, 6, 1, 3, 2, 1, 4, 0, -1, -1, 5, 9, 8, 12, 127, 127};
const int8_t F_R[] PROGMEM = {0, 12, 0, 0, 4, 0, 7, 2, 7, 4, 4, 6, 0, 6, -1, -1, 4, 6, 8, 12, 127, 127};
const int8_t F_S[] PROGMEM = {7, 2, 6, 1, 4, 0, 2, 1, 1, 3, 1, 5, 4, 6, 7, 7, 7, 9, 6, 11, 4, 12, 2, 11, 1, 10, 127, 127};
const int8_t F_T[] PROGMEM = {0, 0, 8, 0, -1, -1, 4, 0, 4, 12, 127, 127};
const int8_t F_U[] PROGMEM = {0, 0, 0, 9, 1, 11, 4, 12, 7, 11, 8, 9, 8, 0, 127, 127};
const int8_t F_V[] PROGMEM = {0, 0, 4, 12, 8, 0, 127, 127};
const int8_t F_W[] PROGMEM = {0, 0, 2, 12, 4, 7, 6, 12, 8, 0, 127, 127};
const int8_t F_X[] PROGMEM = {0, 0, 8, 12, -1, -1, 8, 0, 0, 12, 127, 127};
const int8_t F_Y[] PROGMEM = {0, 0, 4, 6, 8, 0, -1, -1, 4, 6, 4, 12, 127, 127};
const int8_t F_Z[] PROGMEM = {0, 0, 8, 0, 0, 12, 8, 12, 127, 127};
// Umlauts: letter body y 2–12, two short dot ticks at y 0
const int8_t F_AE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 0, 12, 4, 2, 8, 12, -1, -1, 2, 7, 6, 7, 127, 127};
const int8_t F_OE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 4, 2, 6, 3, 7, 5, 8, 7, 7, 9, 6, 11, 4, 12, 2, 11, 1, 9, 0, 7, 1, 5, 2, 3, 4, 2, 127, 127};
const int8_t F_UE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 0, 2, 0, 9, 1, 11, 4, 12, 7, 11, 8, 9, 8, 2, 127, 127};
const int8_t F_SS[] PROGMEM = {0, 12, 0, 0, 3, 0, 6, 1, 7, 3, 7, 5, 4, 6, 7, 7, 7, 10, 5, 12, 0, 12, -1, -1, 0, 6, 4, 6, 127, 127};

// Lookup: 0-25 = A-Z, 26 = Ä, 27 = Ö, 28 = Ü, 29 = ß
const int8_t *const FONT_TABLE[] PROGMEM = {
    F_A, F_B, F_C, F_D, F_E, F_F, F_G, F_H, F_I, F_J,
    F_K, F_L, F_M, F_N, F_O, F_P, F_Q, F_R, F_S, F_T,
    F_U, F_V, F_W, F_X, F_Y, F_Z,
    F_AE, F_OE, F_UE, F_SS};
const int FONT_TABLE_SIZE = 30;
const int8_t FONT_CELL_W = 8;  // x range 0..8
const int8_t FONT_CELL_H = 12; // y range 0..12

// ═══════════════════════════════════════════════════════════════════════════════
//  MORSE CODE DATA  (PROGMEM)
//  Alphabet: '.' = dot, '-' = dash, null-terminated strings
// ═══════════════════════════════════════════════════════════════════════════════
const char MC_A[] PROGMEM = ".-";
const char MC_B[] PROGMEM = "-...";
const char MC_C[] PROGMEM = "-.-.";
const char MC_D[] PROGMEM = "-..";
const char MC_E[] PROGMEM = ".";
const char MC_F[] PROGMEM = "..-.";
const char MC_G[] PROGMEM = "--.";
const char MC_H[] PROGMEM = "....";
const char MC_I[] PROGMEM = "..";
const char MC_J[] PROGMEM = ".---";
const char MC_K[] PROGMEM = "-.-";
const char MC_L[] PROGMEM = ".-..";
const char MC_M[] PROGMEM = "--";
const char MC_N[] PROGMEM = "-.";
const char MC_O[] PROGMEM = "---";
const char MC_P[] PROGMEM = ".--.";
const char MC_Q[] PROGMEM = "--.-";
const char MC_R[] PROGMEM = ".-.";
const char MC_S[] PROGMEM = "...";
const char MC_T[] PROGMEM = "-";
const char MC_U[] PROGMEM = "..-";
const char MC_V[] PROGMEM = "...-";
const char MC_W[] PROGMEM = ".--";
const char MC_X[] PROGMEM = "-..-";
const char MC_Y[] PROGMEM = "-.--";
const char MC_Z[] PROGMEM = "--..";

const char *const MORSE_ABC[] PROGMEM = {
    MC_A, MC_B, MC_C, MC_D, MC_E, MC_F, MC_G, MC_H, MC_I, MC_J,
    MC_K, MC_L, MC_M, MC_N, MC_O, MC_P, MC_Q, MC_R, MC_S, MC_T,
    MC_U, MC_V, MC_W, MC_X, MC_Y, MC_Z};

const char MW_0[] PROGMEM = "LICHT";
const char MW_1[] PROGMEM = "LASER";
const char MW_2[] PROGMEM = "KUNST";
const char MW_3[] PROGMEM = "RAUM";
const char MW_4[] PROGMEM = "WELT";
const char MW_5[] PROGMEM = "NACHT";
const char MW_6[] PROGMEM = "TAG";
const char MW_7[] PROGMEM = "FORM";
const char MW_8[] PROGMEM = "LINIE";
const char MW_9[] PROGMEM = "PUNKT";

const char *const MORSE_WORDS[] PROGMEM = {
    MW_0, MW_1, MW_2, MW_3, MW_4, MW_5, MW_6, MW_7, MW_8, MW_9};
const int MORSE_WORD_COUNT = 10;

// Symbol dimensions (canvas-fraction units)
const float MRS_DOT = 0.015f;  // dot line length
const float MRS_DASH = 0.045f; // dash line length
const float MRS_EGAP = 0.008f; // gap between symbols within a letter
const float MRS_LGAP = 0.022f; // gap between letters

// ─── Day/night rhythm state ──────────────────────────────────────────────────
struct CloudPreset
{
  uint8_t minDim;
  uint16_t fadeDownMs;
  uint16_t holdMs;
  uint16_t fadeUpMs;
  uint8_t scatterReps; // 0 = pick randomly at runtime
};
const CloudPreset CLOUD_PRESETS[] = {
    {200, 500, 600, 700, 1},    // thin wispy
    {130, 1500, 2000, 1800, 1}, // fluffy cumulus
    {70, 2500, 3500, 2200, 1},  // big puffy
    {15, 4000, 6000, 3500, 1},  // thick storm cloud
    {170, 250, 350, 300, 0},    // scattered — reps random
};
const int CLOUD_PRESET_COUNT = 5;

enum CloudPhase
{
  CP_SUNNY,
  CP_FADING_OUT,
  CP_COVERED,
  CP_FADING_IN
};
CloudPhase cloudPhase = CP_SUNNY;
CloudPreset cloudCur;
uint8_t cloudScatter = 0;
uint8_t cloudDimmer = 255;
uint8_t cloudFadeFrom = 255;
uint8_t cloudFadeTo = 255;
uint32_t cloudStateMs = 0;
uint32_t cloudSunnyUntil = 0;
uint32_t lastDmxMs = 0;

// Sub-modes cycled during the laser drawing phase
const DrawMode DN_SUBMODES[] = {
    MODE_OUTLINE, MODE_CORNERS, MODE_LETTERS, MODE_CIRCLE, MODE_MORSE};
const int DN_SUBMODE_COUNT = 5;

enum DNPhase
{
  DN_DRAWING,
  DN_SUN
};
DNPhase dnPhase = DN_DRAWING;
uint32_t dnPhaseStart = 0;
DrawMode dnSubMode = MODE_CORNERS;

// ═══════════════════════════════════════════════════════════════════════════════
//  LASER
// ═══════════════════════════════════════════════════════════════════════════════

void setLaser(int power)
{
  laserEnabled = (power > 0);
  analogWrite(LASER_PIN, laserEnabled ? currentLaserPower : 0);
}

// 3 quick laser blinks to confirm a calibration point was saved
void laserConfirmBlink()
{
  for (int i = 0; i < 3; i++)
  {
    setLaser(0);
    delay(80);
    setLaser(currentLaserPower);
    delay(80);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LED BLINK  (non-blocking)
//  Blinks blinkTarget times, then pauses BLINK_PAUSE_MS, then repeats.
//  In READY state: LED is off.
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
    digitalWrite(LED_BUILTIN, LOW);
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
  if (digitalRead(BTN_JOYSTICK) == LOW)
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

void checkModeButtons()
{
  unsigned long now = millis();
  if (digitalRead(BTN_PREV) == LOW && now - lastPrevTime > DEBOUNCE_MS)
  {
    lastPrevTime = now;
    setDrawMode((DrawMode)((currentDrawMode - 1 + DRAW_MODE_COUNT) % DRAW_MODE_COUNT));
  }
  if (digitalRead(BTN_NEXT) == LOW && now - lastNextTime > DEBOUNCE_MS)
  {
    lastNextTime = now;
    setDrawMode((DrawMode)((currentDrawMode + 1) % DRAW_MODE_COUNT));
  }
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
  float sx = potiToSpeed(analogRead(POTI1));
  float sy = potiToSpeed(analogRead(POTI2));
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
  setLaser(currentLaserPower);
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

void handleCalibrationState()
{
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
  case MODE_CORNERS:
    Serial.println(F("MODE: CORNERS"));
    break;
  case MODE_LETTERS:
    Serial.println(F("MODE: LETTERS"));
    break;
  case MODE_CIRCLE:
    Serial.println(F("MODE: CIRCLE"));
    break;
  case MODE_MANUAL:
    Serial.println(F("MODE: MANUAL"));
    moveToCanvas(0.5f, 0.5f); // start at canvas centre
    setLaser(currentLaserPower);
    break;
  case MODE_SERIAL:
    Serial.println(F("MODE: SERIAL"));
    break;
  case MODE_MORSE:
    Serial.println(F("MODE: MORSE"));
    break;
  case MODE_DAYNIGHT:
    dnPhase = DN_DRAWING;
    dnPhaseStart = millis();
    dnSubMode = DN_SUBMODES[random(DN_SUBMODE_COUNT)];
    cloudEnterSunny();
    Serial.println(F("MODE: DAYNIGHT (5min laser / 1min sun)"));
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

// Returns true when the day/night phase timer has expired and a switch is due.
bool dnShouldSwitch()
{
  if (currentDrawMode != MODE_DAYNIGHT)
    return false;
  uint32_t elapsed = millis() - dnPhaseStart;
  return (dnPhase == DN_DRAWING) ? (elapsed >= DN_LASER_MS) : (elapsed >= DN_SUN_MS);
}

void waitForMotors()
{
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0)
  {
    xStepper.run();
    yStepper.run();
    // During day/night laser phase: keep DMX fixture at full sun ~33 Hz
    if (currentDrawMode == MODE_DAYNIGHT && dnPhase == DN_DRAWING)
    {
      uint32_t now = millis();
      if (now - lastDmxMs >= 30)
      {
        lastDmxMs = now;
        dmxSend(255);
        if (dnShouldSwitch())
        {
          xStepper.stop();
          yStepper.stop();
          break;
        }
      }
    }
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

// Move to position with laser already on — used for multi-segment shapes
void drawSegTo(float nx, float ny)
{
  xStepper.moveTo(normToStepsX(nx));
  yStepper.moveTo(normToStepsY(ny));
  waitForMotors();
}

// Trace the full canvas rectangle once. Checks for mode changes after each side.
#define OUTLINE_CHECK()                                                      \
  do                                                                         \
  {                                                                          \
    if (currentDrawMode != MODE_OUTLINE && currentDrawMode != MODE_DAYNIGHT) \
    {                                                                        \
      setLaser(0);                                                           \
      return;                                                                \
    }                                                                        \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch())                \
    {                                                                        \
      setLaser(0);                                                           \
      return;                                                                \
    }                                                                        \
  } while (0)

void traceCanvasOutline()
{
  moveToCanvas(0.0f, 0.0f);
  setLaser(currentLaserPower);
  xStepper.moveTo(normToStepsX(1.0f));
  yStepper.moveTo(normToStepsY(0.0f));
  waitForMotors();
  OUTLINE_CHECK(); // → top-right
  xStepper.moveTo(normToStepsX(1.0f));
  yStepper.moveTo(normToStepsY(1.0f));
  waitForMotors();
  OUTLINE_CHECK(); // → bottom-right
  xStepper.moveTo(normToStepsX(0.0f));
  yStepper.moveTo(normToStepsY(1.0f));
  waitForMotors();
  OUTLINE_CHECK(); // → bottom-left
  xStepper.moveTo(normToStepsX(0.0f));
  yStepper.moveTo(normToStepsY(0.0f));
  waitForMotors(); // → top-left (close)
  setLaser(0);
}

// Draw one random 90° corner shape anywhere on the canvas
void drawRandomCorner()
{
  // Arm lengths: 30–200 canvas units (3–20 % of canvas)
  int lenH = random(30, 201);
  int lenV = random(30, 201);

  // Vertex position — kept away from edges so arms have room
  int vx = random(lenH, 1001 - lenH);
  int vy = random(lenV, 1001 - lenV);

  // Random direction for each arm independently
  int dx = (random(2) == 0) ? lenH : -lenH;
  int dy = (random(2) == 0) ? lenV : -lenV;

  int startX = constrain(vx + dx, 0, 1000);
  int endY = constrain(vy + dy, 0, 1000);

  // Move to start of horizontal arm, then draw to vertex, then draw vertical arm
  moveToCanvas(startX / 1000.0f, vy / 1000.0f);
  setLaser(currentLaserPower);
  drawSegTo(vx / 1000.0f, vy / 1000.0f);   // → vertex
  drawSegTo(vx / 1000.0f, endY / 1000.0f); // → end of vertical arm
  setLaser(0);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FONT RENDERING
// ═══════════════════════════════════════════════════════════════════════════════

// Return PROGMEM pointer for a character, or nullptr if unsupported.
// Uppercase and lowercase map to the same glyphs (A-Z).
const int8_t *getGlyph(char c)
{
  int idx = -1;
  if (c >= 'A' && c <= 'Z')
    idx = c - 'A';
  else if (c >= 'a' && c <= 'z')
    idx = c - 'a';
  else if (c == (char)0xC4 || c == (char)0xE4)
    idx = 26; // Ä / ä
  else if (c == (char)0xD6 || c == (char)0xF6)
    idx = 27; // Ö / ö
  else if (c == (char)0xDC || c == (char)0xFC)
    idx = 28; // Ü / ü
  else if (c == (char)0xDF)
    idx = 29; // ß
  if (idx < 0)
    return nullptr;
  return (const int8_t *)pgm_read_word(&FONT_TABLE[idx]);
}

// Draw a single glyph.
//   ox, oy : canvas origin of the character cell (0.0–1.0)
//   scale  : canvas units per font grid unit (e.g. 0.03 → char ~15% wide)
void drawGlyph(float ox, float oy, float scale, const int8_t *glyph)
{
  if (!glyph)
    return;
  bool needsMove = true;
  int idx = 0;
  setLaser(0);

  while (true)
  {
    int8_t gx = (int8_t)pgm_read_byte(&glyph[idx]);
    int8_t gy = (int8_t)pgm_read_byte(&glyph[idx + 1]);
    idx += 2;

    if (gx == 127)
      break; // end of glyph
    if (gx == -1)
    { // pen up — lift laser, next point is a move
      setLaser(0);
      needsMove = true;
      continue;
    }

    float cx = ox + gx * scale;
    float cy = oy + (FONT_CELL_H - gy) * scale; // flip y: canvas y-axis is inverted vs font definition

    if (needsMove)
    {
      moveToCanvas(cx, cy);
      needsMove = false;
    }
    else
    {
      if (!laserEnabled)
        setLaser(currentLaserPower);
      drawSegTo(cx, cy);
    }
  }
  setLaser(0);
}

// Draw one random German character at a random position with a random size.
void drawRandomLetter()
{
  // Scale: font cell unit → canvas fraction. 0.015–0.055 → char 7–28% of canvas wide.
  float scale = random(15, 56) / 1000.0f;
  float charW = FONT_CELL_W * scale;
  float charH = FONT_CELL_H * scale;

  // Random origin, clamped so the full glyph stays inside the canvas
  long maxOx = max(1L, (long)((1.0f - charW) * 1000));
  long maxOy = max(1L, (long)((1.0f - charH) * 1000));
  float ox = random(0, maxOx) / 1000.0f;
  float oy = random(0, maxOy) / 1000.0f;

  // Pick from the full German set (A-Z + Ä Ö Ü ß)
  int idx = random(FONT_TABLE_SIZE);
  const char *LETTER_NAMES[] = {
    "A","B","C","D","E","F","G","H","I","J","K","L","M",
    "N","O","P","Q","R","S","T","U","V","W","X","Y","Z",
    "Ä","Ö","Ü","ß"
  };
  Serial.println(LETTER_NAMES[idx]);
  drawGlyph(ox, oy, scale, (const int8_t *)pgm_read_word(&FONT_TABLE[idx]));
}

// Continuously draw a circle centred on the canvas.
// 24 segments (15° each) for a smooth round result.
#define CIRCLE_CHECK()                                                      \
  do                                                                        \
  {                                                                         \
    if (currentDrawMode != MODE_CIRCLE && currentDrawMode != MODE_DAYNIGHT) \
    {                                                                       \
      setLaser(0);                                                          \
      return;                                                               \
    }                                                                       \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch())               \
    {                                                                       \
      setLaser(0);                                                          \
      return;                                                               \
    }                                                                       \
  } while (0)

void drawCircleMode()
{
  const int SEG = 24;
  const float CX = 0.5f;
  const float CY = 0.5f;
  const float R = 0.38f; // radius as fraction of canvas

  moveToCanvas(CX + R, CY); // start at 3 o'clock, laser off
  setLaser(currentLaserPower);
  for (int i = 1; i <= SEG; i++)
  {
    float a = (float)i * 2.0f * PI / SEG;
    drawSegTo(CX + R * cos(a), CY + R * sin(a));
    if (i == SEG / 2)
      CIRCLE_CHECK(); // interruptible at halfway point
  }
  setLaser(0);
}

/// Freehand draw: same velocity jog as calibration, but clamped to canvas bounds.
void handleManualMode()
{
  float sx = potiToSpeed(analogRead(POTI1));
  float sy = potiToSpeed(analogRead(POTI2));

  // Stop each axis at the canvas edge instead of running past it
  long xPos = xStepper.currentPosition();
  long yPos = yStepper.currentPosition();
  if (sx < 0 && xPos <= 0)
    sx = 0;
  if (sx > 0 && xPos >= canvas_width_steps)
    sx = 0;
  if (sy < 0 && yPos <= 0)
    sy = 0;
  if (sy > 0 && yPos >= canvas_height_steps)
    sy = 0;

  xStepper.setSpeed(sx);
  yStepper.setSpeed(sy);
  xStepper.runSpeed();
  yStepper.runSpeed();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MORSE DRAWING
// ═══════════════════════════════════════════════════════════════════════════════

// Canvas width consumed by one letter's morse code (PROGMEM code string ptr)
float morseLetterWidth(const char *code)
{
  float w = 0.0f;
  bool first = true;
  for (uint8_t i = 0;; i++)
  {
    char c = (char)pgm_read_byte(&code[i]);
    if (c == '\0')
      break;
    if (!first)
      w += MRS_EGAP;
    w += (c == '-') ? MRS_DASH : MRS_DOT;
    first = false;
  }
  return w;
}

// Total canvas width for a full word (PROGMEM word string, uppercase A-Z)
float morseWordWidth(const char *word)
{
  float w = 0.0f;
  bool firstLetter = true;
  for (uint8_t i = 0;; i++)
  {
    char c = (char)pgm_read_byte(&word[i]);
    if (c == '\0')
      break;
    if (c < 'A' || c > 'Z')
      continue;
    const char *code = (const char *)pgm_read_word(&MORSE_ABC[c - 'A']);
    if (!firstLetter)
      w += MRS_LGAP;
    w += morseLetterWidth(code);
    firstLetter = false;
  }
  return w;
}

#define MORSE_CHECK()                                                      \
  do                                                                       \
  {                                                                        \
    if (currentDrawMode != MODE_MORSE && currentDrawMode != MODE_DAYNIGHT) \
    {                                                                      \
      setLaser(0);                                                         \
      return;                                                              \
    }                                                                      \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch())              \
    {                                                                      \
      setLaser(0);                                                         \
      return;                                                              \
    }                                                                      \
  } while (0)

void drawMorseWord(float ox, float oy, const char *word)
{
  float x = ox;
  bool firstLetter = true;
  for (uint8_t i = 0;; i++)
  {
    char c = (char)pgm_read_byte(&word[i]);
    if (c == '\0')
      break;
    if (c < 'A' || c > 'Z')
      continue;
    const char *code = (const char *)pgm_read_word(&MORSE_ABC[c - 'A']);
    if (!firstLetter)
      x += MRS_LGAP;
    firstLetter = false;
    bool firstSym = true;
    for (uint8_t j = 0;; j++)
    {
      char sym = (char)pgm_read_byte(&code[j]);
      if (sym == '\0')
        break;
      if (!firstSym)
        x += MRS_EGAP;
      firstSym = false;
      float symW = (sym == '-') ? MRS_DASH : MRS_DOT;
      moveToCanvas(x, oy);
      setLaser(currentLaserPower);
      drawSegTo(x + symW, oy);
      setLaser(0);
      x += symW;
    }
    MORSE_CHECK();
  }
}

void drawMorseMode()
{
  int wi = random(MORSE_WORD_COUNT);
  const char *word = (const char *)pgm_read_word(&MORSE_WORDS[wi]);
  Serial.print(F("MORSE: "));
  for (uint8_t i = 0;; i++) {
    char c = (char)pgm_read_byte(&word[i]);
    if (c == '\0') break;
    Serial.print(c);
  }
  Serial.println();
  float totalW = morseWordWidth(word);
  float maxOx = 1.0f - totalW;
  if (maxOx < 0.0f)
    maxOx = 0.0f;
  float ox = random(0, max(1, (int)(maxOx * 1000))) / 1000.0f;
  float oy = random(50, 951) / 1000.0f;
  drawMorseWord(ox, oy, word);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DAY/NIGHT RHYTHM  (RS485 DMX + 5-min laser / 1-min sun cycle)
//  Wiring: Mega pin 18 (Serial1 TX) → RS485 DI | pin 2 → RS485 DE+/RE-
// ═══════════════════════════════════════════════════════════════════════════════

void dmxSend(uint8_t val)
{
  // Break: hold TX line low ≥ 88 µs
  Serial1.end();
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  delayMicroseconds(100);
  // Mark After Break: ≥ 8 µs
  digitalWrite(18, HIGH);
  delayMicroseconds(12);
  Serial1.begin(250000, SERIAL_8N2);
  Serial1.write((uint8_t)0x00); // start code
  for (int i = 1; i <= DMX_ADDRESS; i++)
    Serial1.write(i == DMX_ADDRESS ? val : (uint8_t)0);
  Serial1.flush();
}

uint8_t lerpU8(uint8_t from, uint8_t to, uint32_t elapsed, uint16_t duration)
{
  if (elapsed >= duration)
    return to;
  return (uint8_t)(from + (int32_t)(to - from) * (int32_t)elapsed / duration);
}

void cloudEnterSunny()
{
  cloudPhase = CP_SUNNY;
  cloudSunnyUntil = millis() + random(3000, 15001);
}

void cloudStartDip()
{
  cloudFadeFrom = cloudDimmer;
  cloudFadeTo = cloudCur.minDim;
  cloudStateMs = millis();
  cloudPhase = CP_FADING_OUT;
}

void cloudEnterCovered()
{
  cloudStateMs = millis();
  cloudPhase = CP_COVERED;
}

void cloudEnterFadingIn()
{
  cloudFadeFrom = cloudCur.minDim;
  cloudFadeTo = cloudDimmer;
  cloudStateMs = millis();
  cloudPhase = CP_FADING_IN;
}

// Run one tick of the cloud animation and send a DMX frame (rate-limited to ~33 Hz).
void updateDMXCloud(uint8_t sunLevel)
{
  uint32_t now = millis();
  if (now - lastDmxMs < 30)
    return;
  lastDmxMs = now;

  switch (cloudPhase)
  {
  case CP_SUNNY:
    cloudDimmer = sunLevel;
    if (now >= cloudSunnyUntil)
    {
      int idx = random(CLOUD_PRESET_COUNT);
      cloudCur = CLOUD_PRESETS[idx];
      cloudScatter = (cloudCur.scatterReps == 0)
                         ? (uint8_t)random(2, 5)
                         : cloudCur.scatterReps;
      cloudScatter--;
      cloudStartDip();
    }
    break;
  case CP_FADING_OUT:
  {
    uint32_t e = now - cloudStateMs;
    cloudDimmer = lerpU8(cloudFadeFrom, cloudFadeTo, e, cloudCur.fadeDownMs);
    if (e >= cloudCur.fadeDownMs)
      cloudEnterCovered();
    break;
  }
  case CP_COVERED:
    cloudDimmer = cloudCur.minDim;
    if (now - cloudStateMs >= cloudCur.holdMs)
      cloudEnterFadingIn();
    break;
  case CP_FADING_IN:
  {
    uint32_t e = now - cloudStateMs;
    cloudDimmer = lerpU8(cloudFadeFrom, cloudFadeTo, e, cloudCur.fadeUpMs);
    if (e >= cloudCur.fadeUpMs)
    {
      if (cloudScatter > 0)
      {
        cloudScatter--;
        cloudSunnyUntil = now + random(300, 900);
        cloudPhase = CP_SUNNY;
      }
      else
      {
        cloudEnterSunny();
      }
    }
    break;
  }
  }
  dmxSend(cloudDimmer);
}

void handleDayNightMode()
{
  uint32_t now = millis();
  uint8_t sunLevel = 255;

  // Phase transitions
  if (dnPhase == DN_DRAWING && now - dnPhaseStart >= DN_LASER_MS)
  {
    dnPhase = DN_SUN;
    dnPhaseStart = now;
    setLaser(0);
    xStepper.stop();
    yStepper.stop();
    cloudEnterSunny();
    Serial.println(F("DN: SUN"));
    return;
  }
  if (dnPhase == DN_SUN && now - dnPhaseStart >= DN_SUN_MS)
  {
    dnPhase = DN_DRAWING;
    dnPhaseStart = now;
    dnSubMode = DN_SUBMODES[random(DN_SUBMODE_COUNT)];
    Serial.println(F("DN: LASER"));
    return;
  }

  if (dnPhase == DN_SUN)
  {
    updateDMXCloud(sunLevel);
  }
  else
  {
    // waitForMotors() sends DMX at full brightness during blocking draws;
    // send once now for the non-blocking path (corners, letters, morse start).
    if (now - lastDmxMs >= 30)
      dmxSend(sunLevel);
    switch (dnSubMode)
    {
    case MODE_OUTLINE:
      traceCanvasOutline();
      break;
    case MODE_CORNERS:
      drawRandomCorner();
      break;
    case MODE_LETTERS:
      drawRandomLetter();
      break;
    case MODE_CIRCLE:
      drawCircleMode();
      break;
    case MODE_MORSE:
      drawMorseMode();
      break;
    default:
      drawRandomCorner();
      break;
    }
  }
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
  pinMode(BTN_JOYSTICK, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(X_ENABLE, OUTPUT);
  pinMode(Y_ENABLE, OUTPUT);

  digitalWrite(X_ENABLE, LOW); // LOW = enabled on RAMPS
  digitalWrite(Y_ENABLE, LOW);

  pinMode(DMX_PIN, OUTPUT);
  digitalWrite(DMX_PIN, HIGH); // RS485 always in transmit mode

  setLaser(currentLaserPower);

  xStepper.setMaxSpeed(JOG_MAX_SPEED);
  xStepper.setAcceleration(ACCELERATION);
  yStepper.setMaxSpeed(JOG_MAX_SPEED);
  yStepper.setAcceleration(ACCELERATION);

  setBlinkTarget(1);
  Serial.println(F("LASER CANVAS v1.0"));
  Serial.println(F("Jog: potis | hold button to save point"));
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
    checkModeButtons(); // pins 50/51 — prev/next mode
    switch (currentDrawMode)
    {
    case MODE_OUTLINE:
      traceCanvasOutline();
      break;
    case MODE_CORNERS:
      drawRandomCorner();
      break;
    case MODE_LETTERS:
      drawRandomLetter();
      break;
    case MODE_CIRCLE:
      drawCircleMode();
      break;
    case MODE_MANUAL:
      handleManualMode();
      break;
    case MODE_SERIAL:
      handleSerialInput();
      break;
    case MODE_MORSE:
      drawMorseMode();
      break;
    case MODE_DAYNIGHT:
      handleDayNightMode();
      break;
    }
    updateLedBlink();
  }
}
