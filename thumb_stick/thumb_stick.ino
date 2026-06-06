const int PIN_X = A11;
const int PIN_Y = A12;
const int PIN_Z = 21;


void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  int x = analogRead(PIN_X);
  int y = analogRead(PIN_Y);
  int z = digitalRead(PIN_Z);


  // Serial.print("X: ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);

  int x1 = map(x, 0, 1023, 0, 255);
  int y1 = map(y, 0, 1023, 0, 255);

  analogWrite(5, x1);
  analogWrite(6, y1);

  delay(50);
}
