const float Vcc = 3.3;  // Teensy ADC reference voltage (typically 3.3 V)

// Teensy 4.1 analog pins: A0 to A15 map to pins 14–29
const int analogPins[16] = {
  A0, A1, A2, A3, A4, A5, A6, A7,
  A8, A9, A10, A11, A12, A13, A14, A15
};

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  for (int i = 0; i < 16; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);

    Serial.print("A");
    Serial.print(i);
    Serial.print(" - Raw: ");
    Serial.print(raw);
    Serial.print("\tVout: ");
    Serial.print(Vout, 3);
    Serial.print("\t");
  }

  Serial.println();  // Newline after each complete scan
  delay(100);        // 10 Hz sampling rate
}
