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
#define DMX_PIN 4     // RS485 direction-enable (tied DE+/RE-)
                      // RS485 data uses Serial1 TX (pin 18 on Mega)
#define REC_LED_PIN 2 // PWM recording-indicator LED (red)
#define DMX_ADDRESS 1 // fixture base channel (1-based)

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                        TUNABLE  PARAMETERS                               ║
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  MOTORS                                                                   ║
const float JOG_MAX_SPEED  = 100.0;  // steps/sec — calibration jogging
const float DRAW_MAX_SPEED =  12.0;  // steps/sec — detailed drawing (letters, corners)
const float ACCELERATION   = 1000.0; // steps/s²  — applies to all modes
const int   POTI_MIN       = 350;    // joystick dead-zone lower bound (0–1023)
const int   POTI_MAX       = 650;    // joystick dead-zone upper bound (0–1023)
const int CAL_LASER_POWER = 255;     // laser brightness during calibration (0–255)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  RECORDING & PLAYBACK                                                     ║
const uint8_t MAX_RECORDINGS = 20;      // max stored shapes  (RAM: N×POINTS×5 bytes)
const uint8_t MAX_REC_POINTS = 30;      // max waypoints per shape
const uint32_t REC_SAMPLE_MS = 100;     // ms between waypoint samples while recording
const float PLAYBACK_SPEED = 25.0f;     // steps/sec — playback motor speed
const uint8_t PLAYBACK_MIN_NOISE       =  0;  // noise at age 0 (0–1000 canvas units)
const uint8_t PLAYBACK_MAX_NOISE       = 60;  // noise at PLAYBACK_AGE_MAX and beyond
const uint8_t PLAYBACK_RECENCY_WEIGHT  =  1;  // pick bias: 0=flat, 1=linear newest-favoured, higher=stronger
const uint8_t PLAYBACK_FREE_TELLINGS   =  2;  // first N tellings are always noise-free
const uint8_t PLAYBACK_AGE_STEP_MIN    =  1;  // min random age added per telling (after free tellings)
const uint8_t PLAYBACK_AGE_STEP_MAX    =  3;  // max random age added per telling (0 = never ages)
const uint8_t PLAYBACK_AGE_MAX         = 20;  // age value at which noise reaches PLAYBACK_MAX_NOISE
const uint8_t LED_MAX = 70;                // PWM brightness of recording-indicator LED (0–255)
const uint32_t AUTO_CYCLE_MS = 120000UL;    // ms each auto mode runs before cycling to next
const uint32_t REC_IDLE_STOP_MS  = 5000UL; // ms joystick idle → auto-stop recording + go to playback
const uint32_t MANUAL_IDLE_MS = 5000UL;    // ms joystick idle in manual (not recording) → transition to playback / auto mode
const uint32_t IDLE_BREATH_MS    = 4000UL; // period of one idle-breath pulse in manual mode (ms)
const uint8_t  IDLE_MIN_LASER    = 50;     // laser floor during idle breathing (0–255)
const uint8_t INTERMODE_PLAYBACK_MAX = 3; // recordings replayed between mode cycles (0 = none)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  CORNERS MODE                                                             ║
const int CORNER_LEN_MIN = 50;  // min arm length (0–1000 canvas units)
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
const int   LETTER_SCALE_MIN =  6;  // min glyph scale (÷1000 → canvas fraction)
const int   LETTER_SCALE_MAX = 20;  // max glyph scale
const float LETTER_GAP = 0.01f;        // extra spacing between characters (font units)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  PLACE MODE  (dim dot positioning + serial text placement)               ║
const uint8_t PLACE_LASER_POWER = 180; // nearly invisible positioning dot (0–255)
const int PLACE_FONT_SIZE_DEF = 12;    // default font size (÷1000 = scale)
// ╠═══════════════════════════════════════════════════════════════════════════╣
// ║  DAY MODE  (automatic sun cycle — triggers when idle long enough)         ║
const uint32_t DAY_IDLE_MS = 10UL * 60UL * 1000UL; // idle time before day mode triggers (10 min)
const uint32_t DAY_QUIET_MS = 1UL * 60UL * 1000UL; // no-recording window required before trigger (1 min)
const uint32_t DAY_CLOUD_MS = 2UL * 60UL * 1000UL; // duration of cloud simulation phase (2 min)
const uint32_t DAY_FADE_MS = 5000UL;               // laser↔DMX crossfade duration (ms)
const uint8_t  DAY_MIN_LASER = 20;                 // laser never goes below this during day mode (0–255)
// ║  DMX cloud timing (used during day mode cloud phase)                      ║
const uint16_t CLOUD_SUNNY_MIN_MS = 3000;
const uint16_t CLOUD_SUNNY_MAX_MS = 15000;
const uint16_t CLOUD_SCATTER_GAP_MIN = 300;
const uint16_t CLOUD_SCATTER_GAP_MAX = 900;
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
  MODE_CIRCLE,   // sine-wave distorted circle
  MODE_MANUAL,   // poti jog with laser on — freehand / recording
  MODE_PLAYBACK, // replay stored recordings
  MODE_SPIRAL,   // rectangular spiral from canvas centre
  MODE_CORNERS,  // random 90° corner shapes
  MODE_RAIN,     // rainstorm: falling streaks + lightning
  MODE_LETTERS,  // random German characters
  MODE_PLACE     // dim dot positioning + serial text placement
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

// ─── Button debounce / long-press ────────────────────────────────────────────
unsigned long lastPrevTime    = 0;
unsigned long lastNextTime    = 0;
const unsigned long BTN_DEBOUNCE_MS = 20;  // min hold to count as a press on joystick button
const unsigned long MODE_BTN_MS    = 200;  // debounce for prev/next mode buttons
const unsigned long LONG_PRESS_MS  = 700;  // hold threshold for long-press
bool          btnDown         = false;
bool          btnLongFired    = false;
unsigned long btnPressedAt    = 0;

enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };
BtnEvent curBtnEv = BTN_NONE;  // set once per loop tick
bool spotActive = false;       // true while long-press spot mode is held

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
char pendingWord[17] = "";               // word queued for drawing in MODE_LETTERS / MODE_PLACE
int placeFontSize = PLACE_FONT_SIZE_DEF; // current font size for MODE_PLACE (÷1000 = scale)

// ─── Shape recordings ─────────────────────────────────────────────────────────

struct RecPoint { uint16_t x; uint16_t y; uint8_t laser; };
RecPoint recordings[MAX_RECORDINGS][MAX_REC_POINTS];
uint8_t  recLengths[MAX_RECORDINGS];
uint8_t  recAges[MAX_RECORDINGS];        // age counter, incremented after each telling
uint8_t  recTimesPlayed[MAX_RECORDINGS]; // number of completed playbacks per recording
uint8_t  recCount       = 0;
bool     isRecording    = false;
uint8_t  recActiveIdx   = 0;
uint32_t lastSampleMs   = 0;
uint8_t  lastPlayedIdx  = 255;  // 255 = none yet

// ─── Auto-cycle state ────────────────────────────────────────────────────────
uint32_t modeStartMs       = 0;          // millis() when current draw mode started
bool     inInterMode       = false;      // replaying recordings between auto-mode cycles
uint8_t  interModeLeft     = 0;          // recordings remaining in inter-mode phase
DrawMode postInterMode     = MODE_RAIN;  // mode to enter after inter-mode playback
uint32_t lastJoyMoveMs     = 0;          // millis() of last joystick move (idle-stop detection)
bool     autoRecordEnabled = false;      // set on entry to manual from another mode; joystick triggers recording once
bool manualJoyAtRest = true;             // edge-detect: was joystick at rest before this movement?
bool waitingForFirstTouch = false;       // after calibration: laser parked at center, waiting for first joystick move

// ─── Stepper instances ───────────────────────────────────────────────────────
AccelStepper xStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, X_STEP, X_DIR);

// ═══════════════════════════════════════════════════════════════════════════════
//  NEWSTROKE FONT  (KiCad stroke font, ASCII-encoded coordinate pairs)
//  Format: first pair = (left_bearing, right_bearing); subsequent pairs = waypoints.
//  Each char encodes a value: v = char - 'R'  (so 'R'=0, 'A'=-17, '['=9, etc.)
//  Pen-up = two-char marker " R" (0x20 0x52); string ends at null terminator.
//  Cell: x spans [left_bearing … right_bearing], y ≈ [-12 … +10]; y negative = up.
//  Canvas mapping: cx = ox+(gx-lb)*scale,  cy = oy+(NS_MAX_Y-gy)*scale  (canvas y increases upward)
// ═══════════════════════════════════════════════════════════════════════════════
const char NS_A[] PROGMEM = "I[MUWU RK[RFY[";
const char NS_B[] PROGMEM = "G\\SPVQWRXTXWWYVZT[L[LFSFUGVHWJWLVNUOSPLP";
const char NS_C[] PROGMEM = "F[WYVZS[Q[NZLXKVJRJOKKLINGQFSFVGWH";
const char NS_D[] PROGMEM = "G\\L[LFQFTGVIWKXOXRWVVXTZQ[L[";
const char NS_E[] PROGMEM = "H[MPTP RW[M[MFWF";
const char NS_F[] PROGMEM = "HZTPMP RM[MFWF";
const char NS_G[] PROGMEM = "F[VGTFQFNGLIKKJOJRKVLXNZQ[S[VZWYWRSR";
const char NS_H[] PROGMEM = "G]L[LF RLPXP RX[XF";
const char NS_I[] PROGMEM = "MWR[RF";
const char NS_J[] PROGMEM = "JZUFUUTXRZO[M[";
const char NS_K[] PROGMEM = "G\\L[LF RX[OO RXFLR";
const char NS_L[] PROGMEM = "HYW[M[MF";
const char NS_M[] PROGMEM = "F^K[KFRUYFY[";
const char NS_N[] PROGMEM = "G]L[LFX[XF";
const char NS_O[] PROGMEM = "G]PFTFVGXIYMYTXXVZT[P[NZLXKTKMLINGPF";
const char NS_P[] PROGMEM = "G\\L[LFTFVGWHXJXMWOVPTQLQ";
const char NS_Q[] PROGMEM = "G]Z]X\\VZSWQVOV RP[NZLXKTKMLINGPFTFVGXIYMYTXXVZT[P[";
const char NS_R[] PROGMEM = "G\\X[QQ RL[LFTFVGWHXJXMWOVPTQLQ";
const char NS_S[] PROGMEM = "H\\LZO[T[VZWYXWXUWSVRTQPPNOMNLLLJMHNGPFUFXG";
const char NS_T[] PROGMEM = "JZLFXF RR[RF";
const char NS_U[] PROGMEM = "G]LFLWMYNZP[T[VZWYXWXF";
const char NS_V[] PROGMEM = "I[KFR[YF";
const char NS_W[] PROGMEM = "F^IFN[RLV[[F";
const char NS_X[] PROGMEM = "H\\KFY[ RYFK[";
const char NS_Y[] PROGMEM = "I[RQR[ RKFRQYF";
const char NS_Z[] PROGMEM = "H\\KFYFK[Y[";

// Lookup table for A–Z (0–25)
const char *const NS_TABLE[] PROGMEM = {
    NS_A, NS_B, NS_C, NS_D, NS_E, NS_F, NS_G, NS_H, NS_I, NS_J,
    NS_K, NS_L, NS_M, NS_N, NS_O, NS_P, NS_Q, NS_R, NS_S, NS_T,
    NS_U, NS_V, NS_W, NS_X, NS_Y, NS_Z};

// Umlauts: old int8_t format, 8×12 cell — only used for Ä Ö Ü ß fallback
// Pairs of int8_t (x,y) | -1,-1 = pen-up | 127,127 = end of glyph
const int8_t F_AE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 0, 12, 4, 2, 8, 12, -1, -1, 2, 7, 6, 7, 127, 127};
const int8_t F_OE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 4, 2, 6, 3, 7, 5, 8, 7, 7, 9, 6, 11, 4, 12, 2, 11, 1, 9, 0, 7, 1, 5, 2, 3, 4, 2, 127, 127};
const int8_t F_UE[] PROGMEM = {2, 0, 3, 0, -1, -1, 5, 0, 6, 0, -1, -1, 0, 2, 0, 9, 1, 11, 4, 12, 7, 11, 8, 9, 8, 2, 127, 127};
const int8_t F_SS[] PROGMEM = {0, 12, 0, 0, 3, 0, 6, 1, 7, 3, 7, 5, 4, 6, 7, 7, 7, 10, 5, 12, 0, 12, -1, -1, 0, 6, 4, 6, 127, 127};
const int8_t *const UMLAUT_TABLE[] PROGMEM = {F_AE, F_OE, F_UE, F_SS}; // Ä Ö Ü ß

const int FONT_TABLE_SIZE = 30;   // A-Z (0-25) + Ä Ö Ü ß (26-29)
// Newstroke cell constants (y: -12=top, +10=bottom; x: left/right bearing per glyph)
const int8_t NS_MAX_Y    = 12;  // offset added to gy so that y=-12 maps to canvas top
const int8_t NS_CELL_H   = 22;  // full cell height in font units (y=-12..+10)
const int8_t NS_CELL_W   = 21;  // typical cell width (right_bearing - left_bearing)
// Umlaut fallback cell constants (old hand-crafted font)
const int8_t FONT_CELL_W =  8;
const int8_t FONT_CELL_H = 12;

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

// ─── Auto-mode cycle (prev/next buttons skip MODE_MANUAL) ────────────────────
const DrawMode AUTO_MODES[] = {MODE_CIRCLE, MODE_PLAYBACK, MODE_SPIRAL, MODE_CORNERS, MODE_RAIN, MODE_LETTERS};
const int AUTO_MODE_COUNT = 6;

// ─── Day mode state ──────────────────────────────────────────────────────────
enum DayPhase
{
  DAY_NONE,
  DAY_FADE_IN,
  DAY_CLOUDS,
  DAY_FADE_OUT
};
DayPhase dayPhase = DAY_NONE;
uint32_t dayPhaseStart = 0;
uint32_t lastDayModeEndMs = 0; // millis() when last day mode finished (0 = startup)
uint32_t lastRecordingMs = 0;  // millis() when last recording was saved (0 = none yet)
int dayFadeLaserFrom = 0;      // laser power at start of fade-in

// ═══════════════════════════════════════════════════════════════════════════════
//  LASER
// ═══════════════════════════════════════════════════════════════════════════════

// Gamma 2.2 lookup: linear input → perceptually-linear PWM output.
const uint8_t GAMMA_TABLE[256] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6,
    6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12,
    12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19,
    20, 20, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29,
    30, 30, 31, 32, 33, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41,
    42, 43, 43, 44, 45, 46, 47, 48, 49, 49, 50, 51, 52, 53, 54, 55,
    56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
    73, 74, 75, 76, 77, 78, 79, 81, 82, 83, 84, 85, 87, 88, 89, 90,
    91, 93, 94, 95, 97, 98, 99, 100, 102, 103, 105, 106, 107, 109, 110, 111,
    113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 135,
    137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161,
    163, 165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190,
    192, 194, 196, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
    223, 225, 227, 229, 231, 234, 236, 238, 240, 242, 244, 246, 248, 251, 253, 255};

inline uint8_t applyGamma(int val)
{
  if (val <= 0)
    return 0;
  if (val >= 255)
    return 255;
  return pgm_read_byte(&GAMMA_TABLE[val]);
}

void writeLaser(int val)
{
  uint8_t pwm = applyGamma(val);
  analogWrite(LASER_PIN, pwm);
  if (currentState == READY)
    analogWrite(REC_LED_PIN, min((int)pwm, (int)LED_MAX));
}

void setLaser(int power)
{
  laserEnabled = (power > 0);
  writeLaser(laserEnabled ? currentLaserPower : 0);
}

// 3 quick blinks to confirm a calibration point or recording start/stop.
void laserConfirmBlink()
{
  int pwr = (currentState != READY) ? CAL_LASER_POWER : currentLaserPower;
  for (int i = 0; i < 3; i++)
  {
    writeLaser(0);
    laserEnabled = false;
    delay(80);
    writeLaser(pwr);
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

BtnEvent checkButtonEvent()
{
  bool pressed = (digitalRead(BTN_JOYSTICK) == LOW);
  unsigned long now = millis();

  if (pressed && !btnDown)
  {
    btnDown       = true;
    btnLongFired  = false;
    btnPressedAt  = now;
  }
  else if (pressed && btnDown && !btnLongFired)
  {
    if (now - btnPressedAt >= LONG_PRESS_MS)
    {
      btnLongFired = true;
      return BTN_LONG;   // fires while held — immediate feedback
    }
  }
  else if (!pressed && btnDown)
  {
    bool wasLong = btnLongFired;
    btnDown      = false;
    btnLongFired = false;
    if (!wasLong && (now - btnPressedAt) >= BTN_DEBOUNCE_MS)
      return BTN_SHORT;  // fires on release
  }
  return BTN_NONE;
}

void checkModeButtons()
{
  // prev/next buttons deactivated
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
  writeLaser(CAL_LASER_POWER);
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
  Serial.print(F("READY W="));
  Serial.print(canvas_width_steps);
  Serial.print(F(" H="));
  Serial.println(canvas_height_steps);

  // Park laser at canvas center and wait for first joystick touch
  xStepper.setMaxSpeed(JOG_MAX_SPEED);
  yStepper.setMaxSpeed(JOG_MAX_SPEED);
  xStepper.moveTo(normToStepsX(0.5f));
  yStepper.moveTo(normToStepsY(0.5f));
  currentLaserPower = CAL_LASER_POWER;
  writeLaser(currentLaserPower);
  laserEnabled = true;
  waitingForFirstTouch = true;
  Serial.println(F("WAIT: move joystick to begin"));
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
  if (checkButtonEvent() == BTN_SHORT)
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
  if (isRecording)
  {
    isRecording = false;
    recCount++;
  }
  currentDrawMode = m;
  modeStartMs = millis();
  inInterMode = false;
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
    xStepper.setMaxSpeed(JOG_MAX_SPEED);
    yStepper.setMaxSpeed(JOG_MAX_SPEED);
    setLaser(currentLaserPower);
    lastJoyMoveMs     = millis();
    autoRecordEnabled = true;
    manualJoyAtRest   = true;
    Serial.println(F("MODE: MANUAL"));
    break;
  case MODE_PLAYBACK:
    lastPlayedIdx = 255;
    xStepper.setMaxSpeed(PLAYBACK_SPEED);
    yStepper.setMaxSpeed(PLAYBACK_SPEED);
    Serial.print(F("MODE: PLAYBACK ("));
    Serial.print(recCount);
    Serial.println(F(" recordings)"));
    break;
  case MODE_PLACE:
    xStepper.setMaxSpeed(JOG_MAX_SPEED);
    yStepper.setMaxSpeed(JOG_MAX_SPEED);
    writeLaser(PLACE_LASER_POWER);
    laserEnabled = true;
    Serial.print(F("MODE: PLACE (size="));
    Serial.print(placeFontSize);
    Serial.println(F(")"));
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

void waitForMotors()
{
  while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0)
  {
    xStepper.run();
    yStepper.run();
    handleSerialInput();
    if (serialInterrupt)
    {
      xStepper.stop();
      yStepper.stop();
      break;
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
#define RAIN_CHECK()                  \
  do                                  \
  {                                   \
    if (serialInterrupt)              \
    {                                 \
      setLaser(0);                    \
      return;                         \
    }                                 \
    if (currentDrawMode != MODE_RAIN) \
    {                                 \
      setLaser(0);                    \
      return;                         \
    }                                 \
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
#define SPIRAL_CHECK()                  \
  do                                    \
  {                                     \
    if (serialInterrupt)                \
    {                                   \
      setLaser(0);                      \
      return;                           \
    }                                   \
    if (currentDrawMode != MODE_SPIRAL) \
    {                                   \
      setLaser(0);                      \
      return;                           \
    }                                   \
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
#define CIRCLE_CHECK()                  \
  do                                    \
  {                                     \
    if (serialInterrupt)                \
    {                                   \
      setLaser(0);                      \
      return;                           \
    }                                   \
    if (currentDrawMode != MODE_CIRCLE) \
    {                                   \
      setLaser(0);                      \
      return;                           \
    }                                   \
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

// Draw a single newstroke glyph.
//   ox, oy : canvas top-left of the character cell (0.0–1.0)
//   scale  : canvas units per font grid unit
//   data   : PROGMEM pointer to the newstroke string (bearing pair + waypoints)
// Pen-up marker = " R" (0x20 0x52) — two chars; y=-NS_MAX_Y maps to canvas top.
void drawGlyphNS(float ox, float oy, float scale, const char *data)
{
  if (!data) return;
  int8_t lb = (int8_t)((char)pgm_read_byte(&data[0]) - 'R'); // left bearing
  bool needsMove = true;
  setLaser(0);
  int i = 2; // skip bearing pair

  while (true)
  {
    char ca = (char)pgm_read_byte(&data[i]);
    if (ca == '\0') break;

    if (ca == ' ')  // pen-up marker " R" — skip both chars
    {
      setLaser(0);
      needsMove = true;
      i += 2;
      continue;
    }

    char cb = (char)pgm_read_byte(&data[i + 1]);
    if (cb == '\0') break; // guard: stray char at end
    i += 2;

    int8_t gx = (int8_t)(ca - 'R');
    int8_t gy = (int8_t)(cb - 'R');
    float cx = ox + (gx - lb) * scale;
    float cy = oy + (NS_MAX_Y - gy) * scale; // canvas y increases upward; gy=-12=cap→large cy, gy=+9=base→small cy

    if (needsMove)
    {
      moveToCanvas(cx, cy);
      needsMove = false;
    }
    else
    {
      if (!laserEnabled) setLaser(currentLaserPower);
      drawSegTo(cx, cy);
    }
  }
  setLaser(0);
}

// Returns advance width (right_bearing - left_bearing) for a newstroke glyph.
static int8_t glyphAdvance(const char *data)
{
  int8_t lb = (int8_t)((char)pgm_read_byte(&data[0]) - 'R');
  int8_t rb = (int8_t)((char)pgm_read_byte(&data[1]) - 'R');
  return rb - lb;
}

// Umlaut fallback: old int8_t format, 8×12 cell.
// Pairs of int8_t (x,y) | -1,-1 = pen-up | 127,127 = end of glyph
static void drawGlyphOld(float ox, float oy, float scale, const int8_t *glyph)
{
  if (!glyph) return;
  bool needsMove = true;
  int idx = 0;
  setLaser(0);
  while (true)
  {
    int8_t gx = (int8_t)pgm_read_byte(&glyph[idx]);
    int8_t gy = (int8_t)pgm_read_byte(&glyph[idx + 1]);
    idx += 2;
    if (gx == 127) break;
    if (gx == -1) { setLaser(0); needsMove = true; continue; }
    float cx = ox + gx * scale;
    float cy = oy + (FONT_CELL_H - gy) * scale;
    if (needsMove) { moveToCanvas(cx, cy); needsMove = false; }
    else { if (!laserEnabled) setLaser(currentLaserPower); drawSegTo(cx, cy); }
  }
  setLaser(0);
}

void drawSerialWord(const char *word)
{
  int len = strlen(word);
  if (len == 0) return;
  const float GAP = LETTER_GAP;

  // Pre-compute total advance for scaling/centering
  float totalAdv = 0;
  for (int i = 0; i < len; i++)
  {
    char c = word[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
      int nsIdx = (c >= 'A' && c <= 'Z') ? c - 'A' : c - 'a';
      const char *g = (const char *)pgm_read_word(&NS_TABLE[nsIdx]);
      totalAdv += glyphAdvance(g);
    }
    else totalAdv += FONT_CELL_W;
    if (i < len - 1) totalAdv += GAP;
  }

  float scale = min(0.9f / totalAdv, 0.025f);
  float charH = NS_CELL_H * scale;
  float maxOy = max(0.0f, 1.0f - charH);
  float ox = (1.0f - totalAdv * scale) / 2.0f; // horizontally centred
  float oy = random(0, max(1, (int)(maxOy * 1000))) / 1000.0f;

  Serial.print(F("WORD: "));
  Serial.println(word);

  float curX = 0;
  for (int i = 0; i < len; i++)
  {
    char c = word[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
      int nsIdx = (c >= 'A' && c <= 'Z') ? c - 'A' : c - 'a';
      const char *g = (const char *)pgm_read_word(&NS_TABLE[nsIdx]);
      drawGlyphNS(ox + curX * scale, oy, scale, g);
      curX += glyphAdvance(g) + GAP;
    }
    else
    {
      int ui = -1;
      if (c == (char)0xC4 || c == (char)0xE4) ui = 0;
      else if (c == (char)0xD6 || c == (char)0xF6) ui = 1;
      else if (c == (char)0xDC || c == (char)0xFC) ui = 2;
      else if (c == (char)0xDF) ui = 3;
      if (ui >= 0)
      {
        const int8_t *g = (const int8_t *)pgm_read_word(&UMLAUT_TABLE[ui]);
        drawGlyphOld(ox + curX * scale, oy, scale, g);
      }
      curX += FONT_CELL_W + GAP;
    }
  }
  setLaser(0);
}

// Draw a word at an exact canvas position (normalized 0–1) with an explicit scale.
// ox/oy is the top-left origin; text extends right and down from there.
void drawWordAt(const char *word, float ox, float oy, float scale)
{
  int len = strlen(word);
  if (len == 0) return;
  const float GAP = LETTER_GAP;

  Serial.print(F("PLACE: "));
  Serial.print(word);
  Serial.print(F(" @("));
  Serial.print((int)(ox * 1000));
  Serial.print(F(","));
  Serial.print((int)(oy * 1000));
  Serial.print(F(") s="));
  Serial.println(scale);

  xStepper.setMaxSpeed(DRAW_MAX_SPEED);
  yStepper.setMaxSpeed(DRAW_MAX_SPEED);

  float curX = 0;
  for (int i = 0; i < len; i++)
  {
    char c = word[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
      int nsIdx = (c >= 'A' && c <= 'Z') ? c - 'A' : c - 'a';
      const char *g = (const char *)pgm_read_word(&NS_TABLE[nsIdx]);
      drawGlyphNS(ox + curX * scale, oy, scale, g);
      curX += glyphAdvance(g) + GAP;
    }
    else
    {
      int ui = -1;
      if (c == (char)0xC4 || c == (char)0xE4) ui = 0;
      else if (c == (char)0xD6 || c == (char)0xF6) ui = 1;
      else if (c == (char)0xDC || c == (char)0xFC) ui = 2;
      else if (c == (char)0xDF) ui = 3;
      if (ui >= 0)
      {
        const int8_t *g = (const int8_t *)pgm_read_word(&UMLAUT_TABLE[ui]);
        drawGlyphOld(ox + curX * scale, oy, scale, g);
      }
      curX += FONT_CELL_W + GAP;
    }
  }
  setLaser(0);
  xStepper.setMaxSpeed(JOG_MAX_SPEED);
  yStepper.setMaxSpeed(JOG_MAX_SPEED);
}

// Dim-dot positioning + serial text placement.
void handlePlaceMode()
{
  // Keep positioning dot on
  writeLaser(PLACE_LASER_POWER);
  laserEnabled = true;

  // Velocity jog — same edge clamping as manual mode
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

// Draw one random letter (A-Z + Ä Ö Ü ß) at a random position and size.
void drawRandomLetter()
{
  int idx = random(FONT_TABLE_SIZE); // 0-25 = A-Z, 26-29 = umlauts

  float scale, charW, charH;
  if (idx < 26)
  {
    scale = random(LETTER_SCALE_MIN, LETTER_SCALE_MAX + 1) / 1000.0f;
    const char *g = (const char *)pgm_read_word(&NS_TABLE[idx]);
    charW = glyphAdvance(g) * scale;
    charH = NS_CELL_H * scale;
  }
  else
  {
    // Umlauts use the old 8×12 cell; scale so they appear similar in size
    scale = random(LETTER_SCALE_MIN, LETTER_SCALE_MAX + 1) / 1000.0f
            * ((float)NS_CELL_W / FONT_CELL_W);  // compensate for cell size difference
    charW = FONT_CELL_W * scale;
    charH = FONT_CELL_H * scale;
  }

  long maxOx = max(1L, (long)((1.0f - charW) * 1000));
  long maxOy = max(1L, (long)((1.0f - charH) * 1000));
  float ox = random(0, maxOx) / 1000.0f;
  float oy = random(0, maxOy) / 1000.0f;

  const char *LETTER_NAMES[] = {
    "A","B","C","D","E","F","G","H","I","J","K","L","M",
    "N","O","P","Q","R","S","T","U","V","W","X","Y","Z",
    "Ae","Oe","Ue","ss"
  };
  Serial.println(LETTER_NAMES[idx]);

  if (idx < 26)
  {
    const char *g = (const char *)pgm_read_word(&NS_TABLE[idx]);
    drawGlyphNS(ox, oy, scale, g);
  }
  else
  {
    const int8_t *g = (const int8_t *)pgm_read_word(&UMLAUT_TABLE[idx - 26]);
    drawGlyphOld(ox, oy, scale, g);
  }
}

void startRecording()
{
  recActiveIdx = recCount;
  recLengths[recActiveIdx]      = 0;
  recAges[recActiveIdx]         = 0;
  recTimesPlayed[recActiveIdx]  = 0;
  isRecording = true;
  lastSampleMs = millis();
  laserConfirmBlink();
  Serial.print(F("REC start: slot "));
  Serial.println(recCount + 1);
}

/// Freehand draw: same velocity jog as calibration, but clamped to canvas bounds.
void handleManualMode()
{
  // Auto-start recording on first joystick movement — only when freshly entering manual mode
  bool joyMoved = joystickMoved();
  if (!joyMoved)
  {
    manualJoyAtRest = true;
  }
  else if (autoRecordEnabled && manualJoyAtRest && !isRecording && recCount < MAX_RECORDINGS)
  {
    manualJoyAtRest   = false;
    autoRecordEnabled = false;
    startRecording();
  }
  else
  {
    manualJoyAtRest = false;
  }

  if (joyMoved)
    lastJoyMoveMs = millis();

  // Idle auto-stop: joystick still for REC_IDLE_STOP_MS → save and go to playback
  if (isRecording && millis() - lastJoyMoveMs >= REC_IDLE_STOP_MS)
  {
    isRecording = false;
    recCount++;
    lastRecordingMs = millis();
    laserConfirmBlink();
    Serial.print(F("REC idle-saved: "));
    Serial.print(recLengths[recActiveIdx]);
    Serial.println(F(" pts"));
    autoRecordEnabled = false; // re-enabled only when a mode change is interrupted
    return;
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
        lastRecordingMs = millis();
        laserConfirmBlink();
        Serial.println(F("REC auto-stopped (buffer full)"));
        autoRecordEnabled = false; // re-enabled only when a mode change is interrupted
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

  // Idle breathing: slow cosine pulse when joystick at rest and not recording
  if (!isRecording)
  {
    if (!joyMoved)
    {
      float phase  = (float)(millis() % IDLE_BREATH_MS) / (float)IDLE_BREATH_MS;
      float breath = (1.0f - cosf(phase * TWO_PI)) * 0.5f; // 0→1→0
      int   val    = IDLE_MIN_LASER + (int)((currentLaserPower - IDLE_MIN_LASER) * breath);
      writeLaser(val);
      laserEnabled = true;
    }
    else
    {
      writeLaser(currentLaserPower);
      laserEnabled = true;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DAY MODE  (RS485 DMX — automatic idle-triggered sun cycle)
//  Wiring: Mega pin 18 (Serial1 TX) → RS485 DI | pin 4 → RS485 DE+/RE-
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

// ═══════════════════════════════════════════════════════════════════════════════
//  DAY MODE  (automatic — triggers after DAY_IDLE_MS of no recording activity)
//  Sequence: crossfade laser→DMX  →  cloud simulation  →  crossfade DMX→laser
// ═══════════════════════════════════════════════════════════════════════════════

void startDayMode()
{
  xStepper.stop();
  yStepper.stop();
  serialInterrupt = true;
  dayFadeLaserFrom = currentLaserPower;
  xStepper.setMaxSpeed(DRAW_MAX_SPEED);
  yStepper.setMaxSpeed(DRAW_MAX_SPEED);
  xStepper.moveTo(normToStepsX(0.5f));
  yStepper.moveTo(normToStepsY(0.5f));
  dayPhase = DAY_FADE_IN;
  dayPhaseStart = millis();
  Serial.println(F("DAY MODE: fade in"));
}

void handleDayMode()
{
  uint32_t now = millis();
  uint32_t elapsed = now - dayPhaseStart;

  switch (dayPhase)
  {
  case DAY_FADE_IN:
  {
    xStepper.run();
    yStepper.run();
    float t = min(1.0f, (float)elapsed / (float)DAY_FADE_MS);
    int laserVal = DAY_MIN_LASER + (int)((dayFadeLaserFrom - DAY_MIN_LASER) * (1.0f - t));
    writeLaser(laserVal);
    laserEnabled = true;
    if (now - lastDmxMs >= 30)
    {
      lastDmxMs = now;
      dmxSend((uint8_t)(255.0f * t));
    }
    if (elapsed >= DAY_FADE_MS && xStepper.distanceToGo() == 0 && yStepper.distanceToGo() == 0)
    {
      writeLaser(DAY_MIN_LASER);
      laserEnabled = true;
      dmxSend(255);
      lastDmxMs = now;
      cloudEnterSunny();
      dayPhase = DAY_CLOUDS;
      dayPhaseStart = now;
      Serial.println(F("DAY MODE: clouds"));
    }
    break;
  }
  case DAY_CLOUDS:
    writeLaser(DAY_MIN_LASER);
    laserEnabled = true;
    updateDMXCloud(255);
    if (elapsed >= DAY_CLOUD_MS)
    {
      dayPhase = DAY_FADE_OUT;
      dayPhaseStart = now;
      Serial.println(F("DAY MODE: fade out"));
    }
    break;
  case DAY_FADE_OUT:
  {
    float t = min(1.0f, (float)elapsed / (float)DAY_FADE_MS);
    if (now - lastDmxMs >= 30)
    {
      lastDmxMs = now;
      dmxSend((uint8_t)(255.0f * (1.0f - t)));
    }
    int laserVal = DAY_MIN_LASER + (int)((currentLaserPower - DAY_MIN_LASER) * t);
    writeLaser(laserVal);
    laserEnabled = true;
    if (elapsed >= DAY_FADE_MS)
    {
      dmxSend(0);
      writeLaser(0);
      laserEnabled = false;
      dayPhase = DAY_NONE;
      lastDayModeEndMs = now;
      Serial.println(F("DAY MODE: end"));
    }
    break;
  }
  default:
    break;
  }
}

bool checkDayModeTrigger()
{
  if (dayPhase != DAY_NONE)
    return false;
  if (currentDrawMode == MODE_MANUAL)
    return false;
  uint32_t now = millis();
  if (now - lastDayModeEndMs < DAY_IDLE_MS)
    return false;
  if (lastRecordingMs > 0 && now - lastRecordingMs < DAY_QUIET_MS)
    return false;
  startDayMode();
  return true;
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
      total += (uint16_t)(1 + (uint16_t)i * PLAYBACK_RECENCY_WEIGHT);

  uint16_t roll = (uint16_t)random(total);
  uint16_t acc  = 0;
  for (uint8_t i = 0; i < recCount; i++) {
    if (recCount > 1 && i == lastPlayedIdx) continue;
    acc += (uint16_t)(1 + (uint16_t)i * PLAYBACK_RECENCY_WEIGHT);
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

  bool    isFree    = (recTimesPlayed[idx] < PLAYBACK_FREE_TELLINGS);
  float   ageF      = isFree ? 0.0f
                             : constrain((float)recAges[idx] / (float)PLAYBACK_AGE_MAX, 0.0f, 1.0f);
  int16_t noiseScale = (int16_t)(PLAYBACK_MIN_NOISE + ageF * (PLAYBACK_MAX_NOISE - PLAYBACK_MIN_NOISE));

  Serial.print(F("PLAY rec "));
  Serial.print(idx);
  Serial.print(F(" age="));
  Serial.print(recAges[idx]);
  Serial.print(F(" plays="));
  Serial.print(recTimesPlayed[idx]);
  Serial.print(F(" noise="));
  Serial.println(noiseScale);

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

  // Telling complete — update play count and maybe age
  recTimesPlayed[idx]++;
  if (recTimesPlayed[idx] > PLAYBACK_FREE_TELLINGS && PLAYBACK_AGE_STEP_MAX > 0)
    recAges[idx] = (uint8_t)min(255, (int)recAges[idx] + (int)random(PLAYBACK_AGE_STEP_MIN, PLAYBACK_AGE_STEP_MAX + 1));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND INTERFACE  (always active in READY state)
//
//  All successful commands reply "OK\n".
// ═══════════════════════════════════════════════════════════════════════════════

void reportStatus()
{
  uint32_t now = millis();

  // ── Mode ──────────────────────────────────────────────────────────────────
  const __FlashStringHelper *modeName;
  switch (currentDrawMode)
  {
  case MODE_CIRCLE:   modeName = F("CIRCLE");   break;
  case MODE_MANUAL:   modeName = F("MANUAL");   break;
  case MODE_PLAYBACK: modeName = F("PLAYBACK"); break;
  case MODE_SPIRAL:   modeName = F("SPIRAL");   break;
  case MODE_CORNERS:  modeName = F("CORNERS");  break;
  case MODE_RAIN:     modeName = F("RAIN");     break;
  case MODE_LETTERS:  modeName = F("LETTERS");  break;
  case MODE_PLACE:
    modeName = F("PLACE");
    break;
  default:            modeName = F("?");        break;
  }
  Serial.print(F("MODE     "));
  Serial.print(modeName);
  if (inInterMode) Serial.print(F("  [inter-mode]"));
  if (isRecording) Serial.print(F("  [REC]"));
  Serial.println();

  // ── Mode running time / time until next cycle ──────────────────────────────
  uint32_t modeSec = (now - modeStartMs) / 1000UL;
  Serial.print(F("RUN      "));
  Serial.print(modeSec);
  Serial.print(F("s"));
  if (!inInterMode && currentDrawMode != MODE_MANUAL && currentDrawMode != MODE_SPIRAL)
  {
    long secsLeft = ((long)AUTO_CYCLE_MS - (long)(now - modeStartMs)) / 1000L;
    if (secsLeft < 0) secsLeft = 0;
    Serial.print(F("  /  next in "));
    Serial.print(secsLeft);
    Serial.print(F("s"));
  }
  else if (inInterMode)
  {
    Serial.print(F("  /  "));
    Serial.print(interModeLeft);
    Serial.print(F(" rec(s) left"));
  }
  Serial.println();

  // ── Day mode ──────────────────────────────────────────────────────────────
  if (dayPhase != DAY_NONE)
  {
    const __FlashStringHelper *ph;
    switch (dayPhase)
    {
    case DAY_FADE_IN:  ph = F("FADE_IN");  break;
    case DAY_CLOUDS:   ph = F("CLOUDS");   break;
    case DAY_FADE_OUT: ph = F("FADE_OUT"); break;
    default:           ph = F("?");        break;
    }
    Serial.print(F("DAY      "));
    Serial.print(ph);
    Serial.print(F("  "));
    Serial.print((now - dayPhaseStart) / 1000UL);
    Serial.println(F("s elapsed"));
  }
  else if (currentDrawMode == MODE_MANUAL)
  {
    Serial.println(F("DAY      -- (suppressed in MANUAL)"));
  }
  else
  {
    long secsUntil = ((long)DAY_IDLE_MS - (long)(now - lastDayModeEndMs)) / 1000L;
    if (secsUntil <= 0)
    {
      if (lastRecordingMs > 0 && now - lastRecordingMs < DAY_QUIET_MS)
      {
        long recWait = ((long)DAY_QUIET_MS - (long)(now - lastRecordingMs)) / 1000L;
        Serial.print(F("DAY      idle — rec quiet "));
        Serial.print(recWait);
        Serial.println(F("s"));
      }
      else
      {
        Serial.println(F("DAY      imminent"));
      }
    }
    else
    {
      Serial.print(F("DAY      in "));
      Serial.print(secsUntil);
      Serial.println(F("s"));
    }
  }

  // ── Recordings ────────────────────────────────────────────────────────────
  Serial.print(F("REC      "));
  Serial.print(recCount);
  Serial.print(F("/"));
  Serial.print((int)MAX_RECORDINGS);
  if (isRecording)
  {
    Serial.print(F("  [active: "));
    Serial.print(recLengths[recActiveIdx]);
    Serial.print(F(" pts]"));
  }
  Serial.println();

  // ── Laser + canvas + position ──────────────────────────────────────────────
  Serial.print(F("LASER    "));
  Serial.println(currentLaserPower);
  Serial.print(F("CANVAS   "));
  Serial.print(canvas_width_steps);
  Serial.print(F("x"));
  Serial.print(canvas_height_steps);
  Serial.print(F("  pos "));
  Serial.print(xStepper.currentPosition());
  Serial.print(F(","));
  Serial.println(yStepper.currentPosition());
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
    writeLaser(laserEnabled ? currentLaserPower : 0);
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
  else if (currentDrawMode == MODE_PLACE)
  {
    // Numeric input → font size; anything else → draw word at current position
    bool isNum = (strlen(cmd) > 0);
    for (int i = 0; cmd[i] != '\0'; i++)
      if (cmd[i] < '0' || cmd[i] > '9')
      {
        isNum = false;
        break;
      }

    if (isNum)
    {
      placeFontSize = constrain(atoi(cmd), 1, 200);
      Serial.print(F("PLACE SIZE: "));
      Serial.println(placeFontSize);
    }
    else if (strlen(cmd) > 0)
    {
      strncpy(pendingWord, cmd, 16);
      pendingWord[16] = '\0';
      serialInterrupt = true;
    }
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
//  AUTO-CYCLE
// ═══════════════════════════════════════════════════════════════════════════════

// Advance to the next mode in the AUTO_MODES rotation.
// If recordings exist, insert an inter-mode playback phase first.
void advanceAutoMode()
{
  int idx = 0;
  for (int i = 0; i < AUTO_MODE_COUNT; i++)
    if (AUTO_MODES[i] == currentDrawMode)
    {
      idx = i;
      break;
    }
  DrawMode next = AUTO_MODES[(idx + 1) % AUTO_MODE_COUNT];

  if (recCount > 0 && INTERMODE_PLAYBACK_MAX > 0 && currentDrawMode != MODE_PLAYBACK)
  {
    postInterMode = next;
    uint8_t n = (uint8_t)random(1, min((int)recCount, (int)INTERMODE_PLAYBACK_MAX) + 1);
    setDrawMode(MODE_PLAYBACK); // clears inInterMode
    inInterMode = true;         // set AFTER setDrawMode
    interModeLeft = n;
    Serial.print(F("INTER: "));
    Serial.print(n);
    Serial.println(F(" rec(s)"));
  }
  else
  {
    setDrawMode(next);
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
  pinMode(REC_LED_PIN, OUTPUT);
  writeLaser(CAL_LASER_POWER);
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
    handleSerialInput();
    checkModeButtons();
    curBtnEv = checkButtonEvent();

    // Long press: enter spot mode (laser on, brightness from Y joystick, motors stopped)
    if (curBtnEv == BTN_LONG && !spotActive)
    {
      spotActive = true;
      xStepper.stop();
      yStepper.stop();
      serialInterrupt = true;
    }
    // Spot mode end: button released after long press
    if (spotActive && !(btnDown && btnLongFired))
    {
      spotActive = false;
      setLaser(0);
    }

    // First-touch gate: laser parked at center after calibration, waiting for joystick
    if (waitingForFirstTouch)
    {
      xStepper.run();
      yStepper.run();
      if (serialInterrupt)
      {
        // A serial command already set a mode — exit wait and let it run
        waitingForFirstTouch = false;
        writeLaser(0);
        laserEnabled = false;
      }
      else if (joystickMoved())
      {
        waitingForFirstTouch = false;
        setDrawMode(MODE_MANUAL);
      }
      else
      {
        return;
      }
    }

    // Spot mode: laser brightness mapped from Y joystick, motors already stopped
    if (spotActive)
    {
      float spd = potiToSpeed(analogRead(POTI2));
      if (spd < 0)
        spd = -spd;
      writeLaser((int)(spd / JOG_MAX_SPEED * 255.0f));
      laserEnabled = true;
    }
    // Day mode runs its own non-blocking state machine
    else if (dayPhase != DAY_NONE)
    {
      // Joystick cancels day mode
      if (joystickMoved())
      {
        dayPhase = DAY_NONE;
        xStepper.stop();
        yStepper.stop();
        dmxSend(0);
        writeLaser(0);
        laserEnabled = false;
        lastDayModeEndMs = millis();
      }
      else
      {
        handleDayMode();
      }
    }
    else if (!serialInterrupt)
    {
      // Check whether idle timer has elapsed and trigger day mode
      if (!checkDayModeTrigger())
      {
        switch (currentDrawMode)
        {
        case MODE_RAIN:
          drawRainMode();
          break;
        case MODE_SPIRAL:
          drawSpiralMode();
          // spiral returns when it reaches the canvas edge — advance immediately
          if (!serialInterrupt && currentDrawMode == MODE_SPIRAL)
            advanceAutoMode();
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
        case MODE_PLAYBACK:
          handlePlaybackMode();
          if (!serialInterrupt && inInterMode)
          {
            if (--interModeLeft == 0)
            {
              inInterMode = false;
              setDrawMode(postInterMode);
            }
          }
          break;
        case MODE_PLACE:
          handlePlaceMode();
          break;
        }
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
      if (currentDrawMode == MODE_PLACE)
      {
        float nx = (float)(xStepper.currentPosition() - X_left) / (float)canvas_width_steps;
        float ny = (float)(yStepper.currentPosition() - Y_top) / (float)canvas_height_steps;
        drawWordAt(pendingWord, nx, ny, placeFontSize / 1000.0f);
        // Restore dim positioning dot after drawing
        writeLaser(PLACE_LASER_POWER);
        laserEnabled = true;
      }
      else
      {
        drawSerialWord(pendingWord);
      }
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
