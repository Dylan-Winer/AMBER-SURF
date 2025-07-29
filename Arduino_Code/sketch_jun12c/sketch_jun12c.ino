const int fsrPin = A0;
const float Vcc = 3.3;
const float Rfixed = 22000.0;  // 22kΩ

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(fsrPin);
  float Vfsr = raw * (Vcc / 1023.0);

  if (Vfsr == 0) Vfsr = 0.001;  // Avoid divide-by-zero

  float Rfsr = (Vcc - Vfsr) / Vfsr * Rfixed;

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("  V: ");
  Serial.print(Vfsr, 3);
  Serial.print("  R_fsr: ");
  Serial.println(Rfsr);

  delay(500);
}
