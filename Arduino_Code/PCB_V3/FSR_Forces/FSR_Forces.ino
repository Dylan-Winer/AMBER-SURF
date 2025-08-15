// === FSR pin configuration ===
const int analogPins[4] = {A0, A1, A2, A3};
const char* analogNames[4] = {"FSR1", "FSR2", "FSR3", "FSR4"};

// === Voltage reference (Teensy 3.3V logic) ===
const float Vcc = 3.3;

// === Calibration coefficients (Force = slope * Vout + intercept) ===
const float slope[4] = {
  308.010,  // FSR1
  252.743,  // FSR2
  232.058,  // FSR3
  236.940   // FSR4
};

const float intercept[4] = {
  -413.037, // FSR1
  -339.549, // FSR2
  -313.218, // FSR3
  -319.241  // FSR4
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("FSR Calibration Readout");
  Serial.println("Sensor\tRaw\tVout(V)\tForce(N)");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);
    float Force = slope[i] * Vout + intercept[i];

    // Clamp negatives to 0, and zero-out anything < 2 N
    if (Force < 0.0f) Force = 0.0f;
    if (Force < 2.0f) Force = 0.0f;

    // Print in tab-separated format
    Serial.print(analogNames[i]);
    Serial.print(" ");
    Serial.print(raw);
    Serial.print(" ");
    Serial.print(Vout, 3);
    Serial.print(" ");
    Serial.println(Force, 2);
  }

  Serial.println(); // Blank line between readings
  delay(100); // Adjust as needed
}
