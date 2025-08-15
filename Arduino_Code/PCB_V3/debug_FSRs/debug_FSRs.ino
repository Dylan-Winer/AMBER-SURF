// === Voltage reference (Teensy 3.3V logic) ===
const float Vcc = 3.3;

// === Calibration coefficients (Force = slope * Vout + intercept) ===
const float slope[4] = {
  308.010,  // FSR1 (A0)
  252.743,  // FSR2 (A1)
  232.058,  // FSR3 (A2)
  236.940   // FSR4 (A3)
};

const float intercept[4] = {
  -413.037, // FSR1 (A0)
  -339.549, // FSR2 (A1)
  -313.218, // FSR3 (A2)
  -319.241  // FSR4 (A3)
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("FSR Raw, Vout, and Force Readout");
  Serial.println("Sensor\tRaw\tVout(V)\tForce(N)");
}

void loop() {
  // --- FSR1: A0 ---
  int raw_A0 = analogRead(A0);
  float vout_A0 = raw_A0 * (Vcc / 1023.0);
  float force_A0 = slope[0] * vout_A0 + intercept[0];
  if (force_A0 < 2.0) force_A0 = 0.0;
  Serial.print("FSR1\t");
  Serial.print(raw_A0);
  Serial.print("\t");
  Serial.print(vout_A0, 3);
  Serial.print("\t");
  Serial.println(force_A0, 2);

  // --- FSR2: A1 ---
  int raw_A1 = analogRead(A1);
  float vout_A1 = raw_A1 * (Vcc / 1023.0);
  float force_A1 = slope[1] * vout_A1 + intercept[1];
  if (force_A1 < 2.0) force_A1 = 0.0;
  Serial.print("FSR2\t");
  Serial.print(raw_A1);
  Serial.print("\t");
  Serial.print(vout_A1, 3);
  Serial.print("\t");
  Serial.println(force_A1, 2);

  // --- FSR3: A2 ---
  int raw_A2 = analogRead(A2);
  float vout_A2 = raw_A2 * (Vcc / 1023.0);
  float force_A2 = slope[2] * vout_A2 + intercept[2];
  if (force_A2 < 2.0) force_A2 = 0.0;
  Serial.print("FSR3\t");
  Serial.print(raw_A2);
  Serial.print("\t");
  Serial.print(vout_A2, 3);
  Serial.print("\t");
  Serial.println(force_A2, 2);

  // --- FSR4: A3 ---
  int raw_A3 = analogRead(A3);
  float vout_A3 = raw_A3 * (Vcc / 1023.0);
  float force_A3 = slope[3] * vout_A3 + intercept[3];
  if (force_A3 < 2.0) force_A3 = 0.0;
  Serial.print("FSR4\t");
  Serial.print(raw_A3);
  Serial.print("\t");
  Serial.print(vout_A3, 3);
  Serial.print("\t");
  Serial.println(force_A3, 2);

  Serial.println(); // Blank line between readings
  delay(200); // Adjust as needed
}
