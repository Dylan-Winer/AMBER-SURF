const float Vcc = 3.3;  // Teensy ADC reference voltage (typically 3.3 V) 

// Define analog pins and their names
const int analogPins[7] = {A0, A3, A6, A9, A10, A11, A12};
const int ledPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 28, 29, 33};
const char* analogNames[7] = {"A0", "A3", "A6", "A9", "A10", "A11", "A12"};

// Calibration equations: Force = slope * Vout + intercept
const float slope[7] = {
  233.257,   // A0
  298.334,   // A3
  243.102,   // A6
  297.801,   // A9
  602.777,   // A10
  622.299,   // A11
  435.190    // A12
};

const float intercept[7] = {
  -316.886,  // A0
  -403.587,  // A3A0 
  -328.109,  // A6
  -403.681,  // A9
  -812.439,  // A10
  -841.463,  // A11
  -590.396   // A12
};

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  for (int i = 0; i < 7; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);
    float Force = slope[i] * Vout + intercept[i];

    // Adjust round-down cutoff: 4 N for A10, A11, A12; 2 N for others
    float threshold = (i >= 4) ? 4.0 : 2.0;
    if (Force < threshold) Force = 0.0;

    Serial.print(analogNames[i]);
    Serial.print(" - Raw: ");
    Serial.print(raw);
    Serial.print("\tVout: ");
    Serial.print(Vout, 3);
    Serial.print("\tForce: ");
    Serial.print(Force, 1);
    Serial.print(" N\t");
  }

  Serial.println();  // Newline after each complete scan
  delay(100);        // 10 Hz sampling rate
}
