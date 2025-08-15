const float Vcc = 3.3;  // Teensy ADC reference voltage (typically 3.3 V)

// Define analog pins and their names
const int analogPins[4] = {A0, A1, A2, A3};
const char* analogNames[4] = {"A0", "A1", "A2", "A3"};

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);

    Serial.print(analogNames[i]);
    Serial.print(" - Raw: ");
    Serial.print(raw);
    Serial.print("\tVout: ");
    Serial.print(Vout, 3);
    Serial.print("\t");
  }

  Serial.println();  // Newline after each complete scan
  delay(100);        // 10 Hz sampling rate
}
