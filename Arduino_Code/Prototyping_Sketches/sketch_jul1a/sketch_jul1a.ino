const int fsrPin = A0;           // Analog input connected to op-amp output
const float Vcc = 3.3;           // Teensy ADC reference (typically 3.3 V)

// Linear regression coefficients from calibration
const float a = 0.0034;          // Slope
const float b = 1.2746;          // Intercept

void setup() {
  Serial.begin(9600);            // Start serial communication
}

void loop() {
  int raw = analogRead(fsrPin);                          // Read 10-bit ADC (0–1023)
  float Vout = raw * (Vcc / 1023.0);                     // Convert to volts

  float force = (Vout - b) / a;                          // Solve for Force

  // Round to 0 if force is between -1 and 1
  if (force <= 1.0) {
    force = 0.0;
  }

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.print(Vout, 3);
  Serial.print("\tForce: ");
  Serial.print(force, 2);
  Serial.println(" N");

  delay(100);  // Small delay for readability
}
