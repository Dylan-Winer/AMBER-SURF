const int fsrPin = A0;
const float Vcc = 3.3;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(fsrPin);
  float Vout = raw * (Vcc / 1023.0);
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.println(Vout, 3);
  delay(100);
}
