// DMX512 sunny-day-with-clouds effect — Arduino Uno + TTL→RS485 + ArduinoDMX
// Install via Library Manager: ArduinoDMX  +  ArduinoRS485 (dependency)
//
// Wiring:
//   Uno pin 1 (TX) → RS485 DI
//   Uno pin 2      → RS485 DE + ~RE (tied together)
//   RS485 A/B      → fixture XLR pins 3/2
//   Poti middle    → A0  (outer legs to 5V and GND)
//
// NOTE: pin 1 is shared with USB — upload first, then connect the converter.

#include <ArduinoDMX.h>

#define DMX_ADDRESS   1   // fixture base address
#define DIMMER_OFFSET 0   // dimmer is the first channel

const int POTI = A0;

// ── Cloud types ───────────────────────────────────────────────────────────────
struct Cloud {
  uint8_t  minDim;      // darkest point (0 = black, 255 = full sun)
  uint16_t fadeDownMs;  // time to dim down
  uint16_t holdMs;      // time at darkest
  uint16_t fadeUpMs;    // time to brighten back
  uint8_t  scatterReps; // >1 → fire multiple dips (scattered cloud)
};

const Cloud CLOUDS[] = {
  { 200,  500,  600,  700, 1 },  // thin wispy
  { 130, 1500, 2000, 1800, 1 },  // fluffy cumulus
  {  70, 2500, 3500, 2200, 1 },  // big puffy
  {  15, 4000, 6000, 3500, 1 },  // thick storm cloud
  { 170,  250,  350,  300, 0 },  // scattered — reps set randomly at runtime
};
const int CLOUD_COUNT = 5;

// ── State machine ─────────────────────────────────────────────────────────────
enum State { SUNNY, FADING_OUT, COVERED, FADING_IN };
State     state        = SUNNY;
Cloud     cur;                    // active cloud params
uint8_t   scatterLeft  = 0;       // remaining scatter dips
uint8_t   dimmer       = 255;
uint8_t   sunBrightness = 255;
uint32_t  stateStart   = 0;
uint32_t  sunnyUntil   = 0;       // when to leave SUNNY
uint8_t   fadeFrom, fadeTo;       // dimmer range for current fade

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

// ── Helpers ───────────────────────────────────────────────────────────────────
uint8_t lerpU8(uint8_t from, uint8_t to, uint32_t elapsed, uint16_t duration) {
  if (elapsed >= duration) return to;
  return from + (int32_t)(to - from) * elapsed / duration;
}

void writeDimmer(uint8_t val) {
  DMX.beginTransmission();
  DMX.write(DMX_ADDRESS + DIMMER_OFFSET, val);
  DMX.endTransmission();
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  randomSeed(analogRead(A1));  // float pin for entropy
  DMX.begin(512);
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
          sunnyUntil = now + random(300, 900);  // short gap between scatter dips
          state = SUNNY;
        } else {
          enterSunny();
        }
      }
      break;
    }
  }

  writeDimmer(dimmer);
}
