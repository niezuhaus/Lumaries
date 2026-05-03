const int PIN_X   = A0;
const int PIN_Y   = A1;
const int PIN_BTN = 2;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_BTN, INPUT_PULLUP);
}

void loop() {
  int x       = analogRead(PIN_X);
  int y       = analogRead(PIN_Y);
  bool clicked = digitalRead(PIN_BTN) == LOW;

  // Serial.print("X: ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print("  BTN: ");
  Serial.println(clicked ? "PRESSED" : "released");

  delay(50);
}
