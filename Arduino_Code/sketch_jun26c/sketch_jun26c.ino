const int fsrPin = A0;
const float Vcc = 3.3;         // ADC reference voltage
const float Vref = 0.4231;       // Your custom VREF from resistor divider
const float R_fixed = 100000;  // 100kΩ pull-up resistor

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(fsrPin);
  float Vout = raw * (Vcc / 1023.0);  // Convert to volts

  // Only calculate R_fsr if Vout is valid
  float R_fsr = -1;
  if (Vout > 0 && Vout < Vref) {
    R_fsr = (Vout * R_fixed) / (Vref - Vout);
  }

  // Print values
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.print(Vout, 3);

  Serial.print("\tRfsr: ");
  if (R_fsr > 0) {
    Serial.print(R_fsr, 1);
    Serial.println(" Ω");
  } else {
    Serial.println(" N/A");
  }

  delay(100);
}
