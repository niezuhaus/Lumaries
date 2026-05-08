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
#define POTI1 A12       // X-axis jog
#define POTI2 A11     // Y-axis jog
#define LASER_PIN 45  // PWM
#define DMX_PIN 2     // RS485 direction-enable (tied DE+/RE-)
                      // RS485 data uses Serial1 TX (pin 18 on Mega)
#define DMX_ADDRESS 1 // fixture base channel (1-based)

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                        TUNABLE  PARAMETERS                               ║
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  MOTORS                                                                   ║
const float JOG_MAX_SPEED  = 100.0;  // steps/sec — calibration jogging
const float DRAW_MAX_SPEED =  25.0;  // steps/sec — detailed drawing (letters, corners)
const float ACCELERATION   = 1000.0; // steps/s²  — applies to all modes
const int   POTI_MIN       = 350;    // joystick dead-zone lower bound (0–1023)
const int   POTI_MAX       = 650;    // joystick dead-zone upper bound (0–1023)
const int CAL_LASER_POWER = 255;     // laser brightness during calibration (0–255)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  RECORDING & PLAYBACK                                                     ║
const uint8_t  MAX_RECORDINGS    = 15;  // max stored shapes  (RAM: N×POINTS×5 bytes)
const uint8_t  MAX_REC_POINTS    = 50;  // max waypoints per shape
const uint32_t REC_SAMPLE_MS     = 150; // ms between waypoint samples while recording
const uint8_t  PLAYBACK_MAX_NOISE = 80; // max position jitter for oldest recording (0–1000 units)
                                        // newest = 0 noise, oldest = PLAYBACK_MAX_NOISE
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  CORNERS MODE                                                             ║
const int CORNER_LEN_MIN = 30;  // min arm length (0–1000 canvas units)
const int CORNER_LEN_MAX = 200; // max arm length
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  CIRCLE MODE  (sine-wave distorted circle)                                ║
const int   CIRCLE_SEGMENTS   = 120;   // drawing resolution (more = smoother)
const float CIRCLE_RADIUS     = 0.35f; // base radius as fraction of canvas (0.0–0.5)
const float CIRCLE_SINE_AMP   = 0.08f; // first sine wave amplitude (canvas fraction)
const int   CIRCLE_SINE_FREQ  = 5;     // first sine wave bumps around the circle
const float CIRCLE_SINE_AMP2  = 0.04f; // second sine wave amplitude
const int   CIRCLE_SINE_FREQ2 = 11;    // second sine wave frequency (prime avoids locking with freq1)
const float CIRCLE_PHASE_STEP1 = 0.31f; // radians p1 advances each circle (~1/20 rotation)
const float CIRCLE_PHASE_STEP2 = 0.17f; // radians p2 advances each circle (different rate)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  RAIN MODE  (rainstorm — streaks + lightning flashes)                     ║
const float    RAIN_LINE_LENGTH  = 0.15f;  // streak length as fraction of canvas
const float    RAIN_DRAW_SPEED   = 80.0f;  // steps/sec — fast rain (vs DRAW_MAX_SPEED 25)
const int      RAIN_FLASH_CHANCE = 8;      // 1-in-N chance of lightning after each streak
const int      RAIN_FLASH_COUNT  = 3;      // flash pulses per lightning strike
const uint16_t RAIN_FLASH_ON_MS  = 30;     // ms laser-on per flash pulse
const uint16_t RAIN_FLASH_OFF_MS = 60;     // ms laser-off between flash pulses
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  SPIRAL MODE  (rectangular spiral expanding from centre)                  ║
const float SPIRAL_STEP       = 0.025f; // gap between rings (canvas fraction) — 0.025 → ~20 rings
const float SPIRAL_SPEED_MULT = 3.5f;   // speed multiplier at the canvas edge vs. centre
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  LETTERS MODE                                                             ║
const int   LETTER_SCALE_MIN = 15;  // min glyph scale (÷1000 → canvas fraction)
const int   LETTER_SCALE_MAX = 55;  // max glyph scale
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  DAY/NIGHT MODE                                                           ║
const uint32_t DN_LASER_MS = 5UL * 60UL * 1000UL; // ms of laser drawing per cycle (5 min)
const uint32_t DN_SUN_MS   = 1UL * 60UL * 1000UL; // ms of DMX sun effect per cycle (1 min)
// ║  DMX cloud timing (used inside DN sun phase)                              ║
const uint16_t CLOUD_SUNNY_MIN_MS  = 3000;  // min sunny gap between clouds
const uint16_t CLOUD_SUNNY_MAX_MS  = 15000; // max sunny gap between clouds
const uint16_t CLOUD_SCATTER_GAP_MIN = 300; // ms between scattered cloud repetitions
const uint16_t CLOUD_SCATTER_GAP_MAX = 900; // ms
// ╚═══════════════════════════════════════════════════════════════════════════╝

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
  MODE_CIRCLE,   // 1 — sine-wave distorted circley
  MODE_MANUAL,   // 5 — poti jog with laser on — freehand drawing
  MODE_PLAYBACK, // 6 — replay stored recordings in sequence
  MODE_SPIRAL,   // 2 — rectangular spiral expanding from canvas centre
  MODE_CORNERS,  // 4 — random small 90° corner shapes
  MODE_RAIN,     // 3 — rainstorm: falling streaks + lightning flashes
  MODE_LETTERS,  // 7 — random German characters at random positions
  MODE_DAYNIGHT  // 8 — 5-min laser drawing / 1-min sun DMX cycle
};
DrawMode currentDrawMode = MODE_RAIN;
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

// ─── Serial pending command ───────────────────────────────────────────────────
enum SerialPending { SP_NONE, SP_MOVE, SP_DRAW, SP_HOME };
SerialPending pendingSerial = SP_NONE;
float pendingSerialX = 0.0f, pendingSerialY = 0.0f;
bool serialInterrupt = false;
char pendingWord[17] = "";  // word queued for drawing in MODE_LETTERS

// ─── Shape recordings ─────────────────────────────────────────────────────────

struct RecPoint { uint16_t x; uint16_t y; uint8_t laser; };
RecPoint recordings[MAX_RECORDINGS][MAX_REC_POINTS];
uint8_t  recLengths[MAX_RECORDINGS];
uint8_t  recCount       = 0;
bool     isRecording    = false;
uint8_t  recActiveIdx   = 0;
uint32_t lastSampleMs   = 0;
uint8_t  lastPlayedIdx  = 255;  // 255 = none yet

// ─── Stepper instances ───────────────────────────────────────────────────────
AccelStepper xStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, X_STEP, X_DIR);

// ═══════════════════════════════════════════════════════════════════════════════
//  STROKE FONT  (8-wide × 12-tall cell, y=0 at top)
//  Curves are approximated with ~6 waypoints per semicircle (30° steps).
//  Pairs of int8_t (x,y) | -1,-1 = pen-up | 127,127 = end of glyph
// ═══════════════════════════════════════════════════════════════════════════════
const int8_t F_A[] PROGMEM = {0, 0, 4, 12, 8, 0, -1, -1, 2, 6, 6, 6, 127, 127};
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
const int8_t F_V[] PROGMEM = {0, 12, 4, 0, 8, 12, 127, 127};
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
const DrawMode DN_SUBMODES[] = {MODE_RAIN, MODE_SPIRAL, MODE_CIRCLE, MODE_CORNERS, MODE_LETTERS};
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
  int pwr = (currentState != READY) ? CAL_LASER_POWER : currentLaserPower;
  for (int i = 0; i < 3; i++)
  {
    analogWrite(LASER_PIN, 0);
    laserEnabled = false;
    delay(80);
    analogWrite(LASER_PIN, pwr);
    laserEnabled = true;
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

bool joystickMoved()
{
  int x = analogRead(POTI1), y = analogRead(POTI2);
  return (x < POTI_MIN || x > POTI_MAX || y < POTI_MIN || y > POTI_MAX);
}

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
  analogWrite(LASER_PIN, CAL_LASER_POWER);
  laserEnabled = true;
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

  // Laser off — first mode handles its own move to starting position
  setLaser(0);

  currentState = READY;
  currentDrawMode = MODE_RAIN;
  Serial.print(F("READY W="));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H="));
  Serial.print(canvas_height_steps);
  Serial.println(F(" MODE: RAIN"));
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
  if (isRecording) { isRecording = false; recCount++; }
  currentDrawMode = m;
  switch (m)
  {
  case MODE_RAIN:
    xStepper.setMaxSpeed(RAIN_DRAW_SPEED);
    yStepper.setMaxSpeed(RAIN_DRAW_SPEED);
    Serial.println(F("MODE: RAIN"));
    break;
  case MODE_SPIRAL:
    xStepper.setMaxSpeed(DRAW_MAX_SPEED);
    yStepper.setMaxSpeed(DRAW_MAX_SPEED);
    Serial.println(F("MODE: SPIRAL"));
    break;
  case MODE_CIRCLE:
    xStepper.setMaxSpeed(DRAW_MAX_SPEED);
    yStepper.setMaxSpeed(DRAW_MAX_SPEED);
    Serial.println(F("MODE: CIRCLE"));
    break;
  case MODE_CORNERS:
    xStepper.setMaxSpeed(DRAW_MAX_SPEED);
    yStepper.setMaxSpeed(DRAW_MAX_SPEED);
    Serial.println(F("MODE: CORNERS"));
    break;
  case MODE_LETTERS:
    xStepper.setMaxSpeed(DRAW_MAX_SPEED);
    yStepper.setMaxSpeed(DRAW_MAX_SPEED);
    Serial.println(F("MODE: LETTERS"));
    break;
  case MODE_MANUAL:
    Serial.println(F("MODE: MANUAL"));
    moveToCanvas(0.5f, 0.5f); // start at canvas centre
    setLaser(currentLaserPower);
    break;
  case MODE_DAYNIGHT:
    dnPhase = DN_DRAWING;
    dnPhaseStart = millis();
    dnSubMode = DN_SUBMODES[random(DN_SUBMODE_COUNT)];
    cloudEnterSunny();
    Serial.println(F("MODE: DAYNIGHT (5min laser / 1min sun)"));
    break;
  case MODE_PLAYBACK:
    lastPlayedIdx = 255;
    xStepper.setMaxSpeed(DRAW_MAX_SPEED);
    yStepper.setMaxSpeed(DRAW_MAX_SPEED);
    Serial.print(F("MODE: PLAYBACK ("));
    Serial.print(recCount);
    Serial.println(F(" recordings)"));
    break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DRAWING PRIMITIVES  (position mode, blocking)
// ═══════════════════════════════════════════════════════════════════════════════

long normToStepsX(float n)
{
  return X_left + (long)(constrain(n, 0.0f, 1.0f) * canvas_width_steps);
}

long normToStepsY(float n)
{
  return Y_top + (long)(constrain(n, 0.0f, 1.0f) * canvas_height_steps);
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
    handleSerialInput();
    if (currentDrawMode != MODE_MANUAL && joystickMoved())
    {
      setDrawMode(MODE_MANUAL);
      serialInterrupt = true;
    }
    if (serialInterrupt)
    {
      xStepper.stop();
      yStepper.stop();
      break;
    }
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

// Rainstorm: each call draws one falling streak (vertical or 45°) then
// occasionally fires a lightning flash (laser pulses at current position).
#define RAIN_CHECK() \
  do { \
    if (serialInterrupt) { setLaser(0); return; } \
    if (currentDrawMode != MODE_RAIN && currentDrawMode != MODE_DAYNIGHT) { setLaser(0); return; } \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch()) { setLaser(0); return; } \
  } while (0)

void drawRainMode()
{
  xStepper.setMaxSpeed(RAIN_DRAW_SPEED);
  yStepper.setMaxSpeed(RAIN_DRAW_SPEED);

  // Angle: 0 = straight down, PI/4 = 45° right-leaning (wind-driven)
  float angle = (random(2) == 0) ? 0.0f : (PI / 4.0f);
  float dx = sin(angle) * RAIN_LINE_LENGTH;
  float dy = cos(angle) * RAIN_LINE_LENGTH;

  int maxSX = max(1, (int)((1.0f - dx) * 1000));
  int minSY = (int)(dy * 1000); // start in [dy, 1.0] so end = sy - dy >= 0
  float sx = random(0, maxSX + 1) / 1000.0f;
  float sy = random(minSY, 1001) / 1000.0f;

  moveToCanvas(sx, sy);
  RAIN_CHECK();

  setLaser(currentLaserPower);
  drawSegTo(sx + dx, sy - dy); // fall toward lower y = physical bottom
  setLaser(0);
  RAIN_CHECK();

  // Lightning: flash the DMX fixture (room lights up, laser stays off)
  if (random(RAIN_FLASH_CHANCE) == 0)
  {
    for (int i = 0; i < RAIN_FLASH_COUNT; i++)
    {
      dmxSend(255);
      delay(RAIN_FLASH_ON_MS);
      dmxSend(0);
      if (i < RAIN_FLASH_COUNT - 1)
        delay(RAIN_FLASH_OFF_MS);
    }
    RAIN_CHECK();
  }
}

// Rectangular spiral expanding from canvas centre.
// Path: right→up→left→down, segment length increases by 1 every two turns.
// Laser stays on throughout; only turns off once per spiral for the center reposition.
#define SPIRAL_CHECK() \
  do { \
    if (serialInterrupt) { setLaser(0); return; } \
    if (currentDrawMode != MODE_SPIRAL && currentDrawMode != MODE_DAYNIGHT) { setLaser(0); return; } \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch()) { setLaser(0); return; } \
  } while (0)

void drawSpiralMode()
{
  static const float DX[] = { 1,  0, -1,  0};
  static const float DY[] = { 0,  1,  0, -1};
  const float CX = 0.5f, CY = 0.5f;

  setLaser(currentLaserPower);

  float x = CX, y = CY;
  int   len  = 1;
  int   dir  = 0;
  int   pair = 0;

  while (true)
  {
    float nx = x + DX[dir] * len * SPIRAL_STEP;
    float ny = y + DY[dir] * len * SPIRAL_STEP;
    if (nx < 0.0f || nx > 1.0f || ny < 0.0f || ny > 1.0f) break;

    // Chebyshev distance from centre, normalised 0 (centre) → 1 (edge)
    float dx = x - CX; if (dx < 0) dx = -dx;
    float dy = y - CY; if (dy < 0) dy = -dy;
    float dist = (dx > dy ? dx : dy) * 2.0f;
    float spd  = DRAW_MAX_SPEED * (1.0f + (SPIRAL_SPEED_MULT - 1.0f) * dist);
    xStepper.setMaxSpeed(spd);
    yStepper.setMaxSpeed(spd);

    drawSegTo(nx, ny);
    x = nx; y = ny;
    SPIRAL_CHECK();

    dir = (dir + 1) % 4;
    if (++pair >= 2) { pair = 0; len++; }
  }
  // Spiral reached the edge — restart from centre (mode change via button only)
}

// Double sine-wave distorted circle:
//   r(θ) = CIRCLE_RADIUS + A1*sin(F1*θ + p1) + A2*sin(F2*θ + p2)
// Both phases are randomised each call for continuous variation.
#define CIRCLE_CHECK() \
  do { \
    if (serialInterrupt) { setLaser(0); return; } \
    if (currentDrawMode != MODE_CIRCLE && currentDrawMode != MODE_DAYNIGHT) { setLaser(0); return; } \
    if (currentDrawMode == MODE_DAYNIGHT && dnShouldSwitch()) { setLaser(0); return; } \
  } while (0)

void drawCircleMode()
{
  const float CX = 0.5f;
  const float CY = 0.5f;
  static float p1 = 0.0f;
  static float p2 = 0.0f;
  p1 += CIRCLE_PHASE_STEP1;
  p2 += CIRCLE_PHASE_STEP2;
  float r0 = CIRCLE_RADIUS
           + CIRCLE_SINE_AMP  * sin(p1)
           + CIRCLE_SINE_AMP2 * sin(p2);
  // Laser stays on: drawSegTo keeps it lit during the move to the new start point.
  // On first entry the laser may not be on yet, so force it here.
  setLaser(currentLaserPower);
  drawSegTo(CX + r0, CY);
  CIRCLE_CHECK();
  for (int i = 1; i <= CIRCLE_SEGMENTS; i++)
  {
    float a = (float)i * 2.0f * PI / CIRCLE_SEGMENTS;
    float r = CIRCLE_RADIUS
            + CIRCLE_SINE_AMP  * sin(CIRCLE_SINE_FREQ  * a + p1)
            + CIRCLE_SINE_AMP2 * sin(CIRCLE_SINE_FREQ2 * a + p2);
    drawSegTo(CX + r * cos(a), CY + r * sin(a));
    if (i == CIRCLE_SEGMENTS / 2)
      CIRCLE_CHECK();
  }
  // No setLaser(0) — laser stays on so the next iteration's reposition is also drawn.
}

// Draw one random 90° corner shape anywhere on the canvas
void drawRandomCorner()
{
  int lenH = random(CORNER_LEN_MIN, CORNER_LEN_MAX + 1);
  int lenV = random(CORNER_LEN_MIN, CORNER_LEN_MAX + 1);

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

void drawSerialWord(const char *word)
{
  int len = strlen(word);
  if (len == 0) return;
  // Scale so the whole word fits within 90% of canvas width, capped at single-letter max
  float step  = FONT_CELL_W + 1.0f;  // 1 unit gap between chars
  float scale = min(0.9f / (len * step), 0.05f);
  float charH = FONT_CELL_H * scale;
  float totalW = len * step * scale - 1.0f * scale; // subtract trailing gap
  float maxOy = max(0.0f, 1.0f - charH);
  float ox = (1.0f - totalW) / 2.0f; // horizontally centred
  float oy = random(0, max(1, (int)(maxOy * 1000))) / 1000.0f;

  Serial.print(F("WORD: "));
  Serial.println(word);

  for (int i = 0; i < len; i++)
  {
    const int8_t *g = getGlyph(word[i]);
    if (g) drawGlyph(ox + i * step * scale, oy, scale, g);
  }
  setLaser(0);
}

// Draw one random German character at a random position with a random size.
void drawRandomLetter()
{
  float scale = random(LETTER_SCALE_MIN, LETTER_SCALE_MAX + 1) / 1000.0f;
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

/// Freehand draw: same velocity jog as calibration, but clamped to canvas bounds.
void handleManualMode()
{
  // Button: toggle recording
  if (checkButton())
  {
    if (!isRecording)
    {
      if (recCount < MAX_RECORDINGS)
      {
        recActiveIdx = recCount;
        recLengths[recActiveIdx] = 0;
        isRecording = true;
        lastSampleMs = millis();
        laserConfirmBlink();
        Serial.print(F("REC start: slot "));
        Serial.println(recCount + 1);
      }
      else
      {
        Serial.println(F("REC full (10/10) — use C to recalibrate and clear"));
      }
    }
    else
    {
      isRecording = false;
      recCount++;
      laserConfirmBlink();
      Serial.print(F("REC saved: "));
      Serial.print(recLengths[recActiveIdx]);
      Serial.println(F(" pts"));
    }
  }

  // Sample position + laser state while recording
  if (isRecording)
  {
    uint32_t now = millis();
    if (now - lastSampleMs >= REC_SAMPLE_MS)
    {
      lastSampleMs = now;
      uint8_t idx = recLengths[recActiveIdx];
      if (idx < MAX_REC_POINTS)
      {
        recordings[recActiveIdx][idx].x = (uint16_t)constrain(
            map(xStepper.currentPosition(), X_left, X_left + canvas_width_steps, 0, 1000), 0, 1000);
        recordings[recActiveIdx][idx].y = (uint16_t)constrain(
            map(yStepper.currentPosition(), Y_top, Y_top + canvas_height_steps, 0, 1000), 0, 1000);
        recordings[recActiveIdx][idx].laser = laserEnabled ? 1 : 0;
        recLengths[recActiveIdx]++;
      }
      else
      {
        isRecording = false;
        recCount++;
        laserConfirmBlink();
        Serial.println(F("REC auto-stopped (buffer full)"));
      }
    }
  }

  // Jog with canvas edge clamping
  float sx = potiToSpeed(analogRead(POTI1));
  float sy = potiToSpeed(analogRead(POTI2));
  long xPos = xStepper.currentPosition();
  long yPos = yStepper.currentPosition();
  if (sx < 0 && xPos <= X_left)
    sx = 0;
  if (sx > 0 && xPos >= X_left + canvas_width_steps)
    sx = 0;
  if (sy < 0 && yPos <= Y_top)
    sy = 0;
  if (sy > 0 && yPos >= Y_top + canvas_height_steps)
    sy = 0;
  xStepper.setSpeed(sx);
  yStepper.setSpeed(sy);
  xStepper.runSpeed();
  yStepper.runSpeed();
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
  cloudSunnyUntil = millis() + random(CLOUD_SUNNY_MIN_MS, CLOUD_SUNNY_MAX_MS + 1);
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
        cloudSunnyUntil = now + random(CLOUD_SCATTER_GAP_MIN, CLOUD_SCATTER_GAP_MAX + 1);
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
    case MODE_SPIRAL:
      drawSpiralMode();
      break;
    case MODE_CIRCLE:
      drawCircleMode();
      break;
    case MODE_CORNERS:
      drawRandomCorner();
      break;
    case MODE_LETTERS:
      drawRandomLetter();
      break;
    default:
      drawRandomCorner();
      break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PLAYBACK MODE
// ═══════════════════════════════════════════════════════════════════════════════

// Apply random noise (±noiseScale in 0-1000 units) to a recorded coordinate
static float deformCoord(uint16_t v, int16_t noiseScale)
{
  int16_t n = (noiseScale > 0) ? (int16_t)random(-noiseScale, noiseScale + 1) : 0;
  int16_t out = (int16_t)v + n;
  if (out < 0)    out = 0;
  if (out > 1000) out = 1000;
  return out / 1000.0f;
}

// Weighted random pick: weight[i] = i+1 (newest = highest), skip lastPlayedIdx if >1 rec
static uint8_t pickPlaybackIdx()
{
  // Build total weight, excluding lastPlayedIdx when possible
  uint16_t total = 0;
  for (uint8_t i = 0; i < recCount; i++)
    if (recCount == 1 || i != lastPlayedIdx)
      total += (uint16_t)(i + 1);

  uint16_t roll = (uint16_t)random(total);
  uint16_t acc  = 0;
  for (uint8_t i = 0; i < recCount; i++) {
    if (recCount > 1 && i == lastPlayedIdx) continue;
    acc += (uint16_t)(i + 1);
    if (roll < acc) return i;
  }
  return recCount - 1;
}

void handlePlaybackMode()
{
  if (recCount == 0)
    return;

  uint8_t idx = pickPlaybackIdx();
  lastPlayedIdx = idx;

  RecPoint *rec = recordings[idx];
  uint8_t   len = recLengths[idx];

  // Age 0.0 = newest (recCount-1), 1.0 = oldest (0)
  float age = (recCount > 1) ? (float)(recCount - 1 - idx) / (float)(recCount - 1) : 0.0f;
  // Max noise in 0-1000 units: ±80 at full age
  int16_t noiseScale = (int16_t)(age * PLAYBACK_MAX_NOISE);

  Serial.print(F("PLAY rec "));
  Serial.print(idx);
  Serial.print(F(" age="));
  Serial.print((int)(age * 100));
  Serial.println(F("%"));

  if (len > 0)
  {
    moveToCanvas(deformCoord(rec[0].x, noiseScale), deformCoord(rec[0].y, noiseScale));
    if (serialInterrupt || currentDrawMode != MODE_PLAYBACK) { setLaser(0); return; }

    for (uint8_t i = 1; i < len; i++)
    {
      if (rec[i].laser) setLaser(currentLaserPower); else setLaser(0);
      xStepper.moveTo(normToStepsX(deformCoord(rec[i].x, noiseScale)));
      yStepper.moveTo(normToStepsY(deformCoord(rec[i].y, noiseScale)));
      waitForMotors();
      if (serialInterrupt || currentDrawMode != MODE_PLAYBACK) { setLaser(0); return; }
    }
  }

  setLaser(0);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND INTERFACE  (always active in READY state)
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
    pendingSerialX = x / 1000.0f;
    pendingSerialY = y / 1000.0f;
    pendingSerial = SP_MOVE;
    serialInterrupt = true;
  }
  else if (cmd[0] == 'D' && sscanf(cmd + 1, "%d %d", &x, &y) == 2)
  {
    pendingSerialX = x / 1000.0f;
    pendingSerialY = y / 1000.0f;
    pendingSerial = SP_DRAW;
    serialInterrupt = true;
  }
  else if (cmd[0] == 'H')
  {
    pendingSerial = SP_HOME;
    serialInterrupt = true;
  }
  else if (cmd[0] == 'L' && sscanf(cmd + 1, "%d", &p) == 1)
  {
    currentLaserPower = constrain(p, 0, 255);
    analogWrite(LASER_PIN, laserEnabled ? currentLaserPower : 0);
    Serial.print(F("LASER "));
    Serial.println(currentLaserPower);
  }
  else if (cmd[0] == 'S')
  {
    reportStatus();
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'N')
  {
    setDrawMode((DrawMode)((currentDrawMode + 1) % DRAW_MODE_COUNT));
    serialInterrupt = true;
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'P')
  {
    setDrawMode((DrawMode)((currentDrawMode - 1 + DRAW_MODE_COUNT) % DRAW_MODE_COUNT));
    serialInterrupt = true;
    Serial.println(F("OK"));
  }
  else if (cmd[0] == 'C')
  {
    restartCalibration();
    serialInterrupt = true;
    Serial.println(F("OK"));
  }
  else if (strlen(cmd) > 1 && currentDrawMode == MODE_LETTERS)
  {
    // Multi-char input in letters mode → queue as word to draw
    strncpy(pendingWord, cmd, 16);
    pendingWord[16] = '\0';
    serialInterrupt = true;
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

  analogWrite(LASER_PIN, CAL_LASER_POWER);
  laserEnabled = true;

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
    handleSerialInput();
    if (currentDrawMode != MODE_MANUAL && joystickMoved())
    {
      setDrawMode(MODE_MANUAL);
      serialInterrupt = true;
    }
    if (!serialInterrupt)
    {
      switch (currentDrawMode)
      {
      case MODE_RAIN:
        drawRainMode();
        break;
      case MODE_SPIRAL:
        drawSpiralMode();
        break;
      case MODE_CIRCLE:
        drawCircleMode();
        break;
      case MODE_CORNERS:
        drawRandomCorner();
        break;
      case MODE_LETTERS:
        drawRandomLetter();
        break;
      case MODE_MANUAL:
        handleManualMode();
        break;
      case MODE_DAYNIGHT:
        handleDayNightMode();
        break;
      case MODE_PLAYBACK:
        handlePlaybackMode();
        break;
      }
    }
    // Execute any pending serial movement command
    if (pendingSerial != SP_NONE)
    {
      SerialPending cmd = pendingSerial;
      float cx = pendingSerialX, cy = pendingSerialY;
      pendingSerial = SP_NONE;
      serialInterrupt = false;
      if (cmd == SP_MOVE)
        moveToCanvas(cx, cy);
      else if (cmd == SP_DRAW)
        drawToCanvas(cx, cy, currentLaserPower);
      else if (cmd == SP_HOME)
        moveToCanvas(0.0f, 0.0f);
      Serial.println(F("OK"));
    }
    else if (pendingWord[0] != '\0')
    {
      serialInterrupt = false;
      drawSerialWord(pendingWord);
      pendingWord[0] = '\0';
      Serial.println(F("OK"));
    }
    else
    {
      serialInterrupt = false;
    }
    updateLedBlink();
  }
}
