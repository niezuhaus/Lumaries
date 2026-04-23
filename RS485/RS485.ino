// DMX512 sunny-day-with-clouds effect — Arduino Mega + RAMPS + TTL→RS485
// No extra library needed.
//
// Wiring:
//   Mega pin 18 (TX1) → RS485 DI   (Z_MIN endstop connector on RAMPS)
//   Mega pin 2        → RS485 DE + ~RE (tied together)
//   RS485 A/B         → fixture XLR pins 3/2
//   Poti middle       → A0  (outer legs to 5V and GND)

#define DE_PIN        2
#define TX1_PIN       18   // Serial1 TX

#define DMX_ADDRESS   1    // fixture base address (1-based)
#define DIMMER_OFFSET 0    // channel offset from base address

const int POTI = A0;

// ── DMX output ────────────────────────────────────────────────────────────────
void dmxSend(uint8_t val) {
  // Break: hold TX1 low ≥ 88 µs
  Serial1.end();
  pinMode(TX1_PIN, OUTPUT);
  digitalWrite(TX1_PIN, LOW);
  delayMicroseconds(100);
  // Mark After Break: ≥ 8 µs
  digitalWrite(TX1_PIN, HIGH);
  delayMicroseconds(12);
  Serial1.begin(250000, SERIAL_8N2);

  Serial1.write((uint8_t)0x00);  // start code
  int dimmerCh = DMX_ADDRESS + DIMMER_OFFSET;  // 1-based channel number
  for (int i = 1; i <= dimmerCh; i++)
    Serial1.write(i == dimmerCh ? val : (uint8_t)0);
  Serial1.flush();
}

// ── Cloud types ───────────────────────────────────────────────────────────────
struct Cloud {
  uint8_t  minDim;      // darkest point (0 = black, 255 = full sun)
  uint16_t fadeDownMs;
  uint16_t holdMs;
  uint16_t fadeUpMs;
  uint8_t  scatterReps; // 0 = pick randomly at runtime
};

const Cloud CLOUDS[] = {
  { 200,  500,  600,  700, 1 },  // thin wispy
  { 130, 1500, 2000, 1800, 1 },  // fluffy cumulus
  {  70, 2500, 3500, 2200, 1 },  // big puffy
  {  15, 4000, 6000, 3500, 1 },  // thick storm cloud
  { 170,  250,  350,  300, 0 },  // scattered — reps chosen randomly
};
const int CLOUD_COUNT = 5;

// ── State machine ─────────────────────────────────────────────────────────────
enum State { SUNNY, FADING_OUT, COVERED, FADING_IN };
State    state        = SUNNY;
Cloud    cur;
uint8_t  scatterLeft  = 0;
uint8_t  dimmer       = 255;
uint8_t  sunBrightness = 255;
uint32_t stateStart   = 0;
uint32_t sunnyUntil   = 0;
uint8_t  fadeFrom, fadeTo;

void enterSunny() {
  state      = SUNNY;
  sunnyUntil = millis() + random(3000, 15001);
}

void startDip() {
  fadeFrom   = sunBrightness;
  fadeTo     = cur.minDim;
  stateStart = millis();
  state      = FADING_OUT;
}

void enterCovered() {
  stateStart = millis();
  state      = COVERED;
}

void enterFadingIn() {
  fadeFrom   = cur.minDim;
  fadeTo     = sunBrightness;
  stateStart = millis();
  state      = FADING_IN;
}

uint8_t lerpU8(uint8_t from, uint8_t to, uint32_t elapsed, uint16_t duration) {
  if (elapsed >= duration) return to;
  return from + (int32_t)(to - from) * elapsed / duration;
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);  // USB monitor free now that DMX uses Serial1
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, HIGH);  // always transmit
  randomSeed(analogRead(A1));
  enterSunny();
}

void loop() {
  sunBrightness = map(analogRead(POTI), 0, 1023, 60, 255);
  uint32_t now  = millis();

  switch (state) {
    case SUNNY:
      dimmer = sunBrightness;
      if (now >= sunnyUntil) {
        int idx     = random(CLOUD_COUNT);
        cur         = CLOUDS[idx];
        scatterLeft = (cur.scatterReps == 0) ? random(2, 5) : cur.scatterReps;
        scatterLeft--;
        startDip();
      }
      break;

    case FADING_OUT: {
      uint32_t elapsed = now - stateStart;
      dimmer = lerpU8(fadeFrom, fadeTo, elapsed, cur.fadeDownMs);
      if (elapsed >= cur.fadeDownMs) enterCovered();
      break;
    }

    case COVERED:
      dimmer = cur.minDim;
      if (now - stateStart >= cur.holdMs) enterFadingIn();
      break;

    case FADING_IN: {
      uint32_t elapsed = now - stateStart;
      dimmer = lerpU8(fadeFrom, fadeTo, elapsed, cur.fadeUpMs);
      if (elapsed >= cur.fadeUpMs) {
        if (scatterLeft > 0) {
          scatterLeft--;
          sunnyUntil = now + random(300, 900);
          state = SUNNY;
        } else {
          enterSunny();
        }
      }
      break;
    }
  }

  dmxSend(dimmer);
}
