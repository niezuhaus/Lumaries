// RAMPS 1.4 X-axis pins for NEMA 17
#define STEP_PIN 54
#define DIR_PIN 55
#define ENABLE_PIN 38

#define SENSOR_PIN 12
#define LDR_PIN A3

const int stepsPerRevolution = 200;  // NEMA 17 (1.8° per step)
const int microstepping = 16;        // Set to match your driver jumpers (1, 2, 4, 8, or 16)
const int stepsPerRev = stepsPerRevolution * microstepping;

const int lightThreshold = 800;
const int minDelay = 200;   // Fastest speed (microseconds between steps)
const int maxDelay = 2000;  // Slowest speed
const int potiMin = 480;
const int potiMax = 520;
const int bounceWaitSteps = 50;

int adcPin = A0;
int lastPotiIn = 0;
int potiIn = 0;
int stepDelay = 1000;
int stepsToMove = 0;
bool reverse = false;
int magnet;
int light = 0;
int lightMeasures[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int idx = 0;
int deafSteps = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // RAMPS stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);  // LOW = enabled on RAMPS
}

void loop() {
  if (deafSteps > 0) {
    deafSteps--;
    magnet = HIGH;
    light = 500;
  } else if (potiIn > 520 || potiIn < 480) {
    magnet = digitalRead(SENSOR_PIN);
    light = analogRead(LDR_PIN);
    lightMeasures[index] = light;
    index++;
    index %= 10;

    Serial.print(average());
    Serial.print(" ");
    Serial.println(light);
  }

  potiIn = analogRead(adcPin);

  if (magnet == LOW || average() > lightThreshold) {
    digitalWrite(LED_BUILTIN, HIGH);  // if adcIn > 500, led light
    reverse = !reverse;
    deafSteps = bounceWaitSteps;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (potiIn > potiMax) {
    vel = map(potiIn, 512, 1024, minSpeed, maxSpeed);
    steps = reverse ? -10 : 10;
  } else if (potiIn < 504) {
    vel = map(potiIn, 0, 512, maxSpeed, minSpeed);
    steps = reverse ? 10 : -10;
  }
  myStepper.setSpeed(vel);
  if ((potiIn > potiMax || potiIn < potiMin)) {
    myStepper.step(steps);
    lastPotiIn = potiIn;
  }
}

int average() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += lightMeasures[i];
  }
  return sum / 10;
}