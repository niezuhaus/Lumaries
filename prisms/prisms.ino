#include <AccelStepper.h>

// RAMPS 1.4 X-axis pins for NEMA 17
// #define X_STEP 54
// #define X_DIR 55
// #define X_ENABLE 38
// #define Y_STEP 36
// #define Y_DIR 34
// #define Y_ENABLE 30

#define X_STEP 54
#define X_DIR 55
#define X_ENABLE 38
#define Y_STEP 60
#define Y_DIR 61
#define Y_ENABLE 56

#define SENSOR_PIN 12
#define POTI1 A3
#define POTI2 A9
#define DREH_POTI A4
#define LASER_PIN 45

const int stepsPerRevolution = 200;  // NEMA 17 (1.8° per step)
const int microstepping = 16;
const int stepsPerRev = stepsPerRevolution * microstepping;

const int lightThreshold = 800;
const float minSpeed = 0;    // Steps per second (langsam)
const float maxSpeed = 100;  // Steps per second (schnell)
const int potiMin = 350;
const int potiMax = 650;
const int measureIntervall = 200;

int potiIn = 0;
int potiIn2 = 0;
int drehPoti = 0;
int deafSteps = 0;
int speed1 = 0;
int speed2 = 0;
int laser = 0;

// AccelxStepper im DRIVER-Modus (Step + Dir)
AccelStepper xStepper(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper yStepper(AccelStepper::DRIVER, Y_STEP, Y_DIR);

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(POTI1, INPUT);
  pinMode(POTI2, INPUT);
  pinMode(DREH_POTI, INPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  pinMode(X_ENABLE, OUTPUT);
  pinMode(Y_ENABLE, OUTPUT);
  digitalWrite(X_ENABLE, LOW);  // LOW = enabled on RAMPS
  digitalWrite(Y_ENABLE, LOW);

  xStepper.setMaxSpeed(maxSpeed);
  xStepper.setAcceleration(1000);  // Steps/s²

  yStepper.setMaxSpeed(maxSpeed);
  yStepper.setAcceleration(1000);  // Steps/s²
}

void loop() {

  potiIn = analogRead(POTI1);
  potiIn2 = analogRead(POTI2);
  drehPoti = analogRead(DREH_POTI);
  if (deafSteps > 0) {
    deafSteps--;
  } else {
    laser = min(max(map(drehPoti, 100, 900, 255, 0), 0), 255);
    laser = 255;

    if (potiIn > potiMax) {
      speed1 = map(potiIn, potiMax, 1024, minSpeed, maxSpeed);
      xStepper.setSpeed(speed1);  // Positiv = vorwärts
    } else if (potiIn < potiMin) {
      speed1 = map(potiIn, 0, potiMin, maxSpeed, minSpeed);
      xStepper.setSpeed(-speed1);  // Negativ = rückwärts
    } else {
      xStepper.setSpeed(0);  // Stopp in der Mitte
    }

    if (potiIn2 > potiMax) {
      speed2 = map(potiIn2, potiMax, 1024, minSpeed, maxSpeed);
      yStepper.setSpeed(speed2);  // Positiv = vorwärts
    } else if (potiIn2 < potiMin) {
      speed2 = map(potiIn2, 0, potiMin, maxSpeed, minSpeed);
      yStepper.setSpeed(-speed2);  // Negativ = rückwärts
    } else {
      yStepper.setSpeed(0);  // Stopp in der Mitte
    }

    Serial.print(potiIn);
    Serial.print(' ');
    Serial.print(speed1);
    Serial.print(' ');
    Serial.print(potiIn2);
    Serial.print(' ');
    Serial.print(speed2);
    Serial.print(' ');
    Serial.print(drehPoti);
    Serial.print(' ');
    Serial.print(laser);
    Serial.println(' ');

    deafSteps = measureIntervall;
  }
  analogWrite(LASER_PIN, laser);
  // Motor bewegen(non - blocking)
  xStepper.runSpeed();
  yStepper.runSpeed();
}
