const int fsrPin = A0;
const float Vcc   = 3.3;       // Teensy's ADC reference
const float Vref  = 0.656;     // Measured V_REF from filtered PWM
const float R_fixed = 4660;  // Fixed resistor R1

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(fsrPin);                      // 0–1023
  float Vout = raw * (Vcc / 1023.0);                 // Convert to volts

  float R_fsr = -1;
  if (Vout > 0 && Vout < Vref) {
    R_fsr = (Vout * R_fixed) / (Vref - Vout);        // Standard divider formula
  }

  // Print results
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.print(Vout, 3);
  Serial.print("\tR_fsr: ");
  if (R_fsr > 0)
    Serial.print(R_fsr, 1);
  else
    Serial.print("N/A");
  Serial.println(" Ω");

  delay(100);
}
