const int fsrPin = A0;           // Analog input connected to op-amp output
const float Vcc = 3.3;           // Teensy ADC reference (typically 3.3 V)

void setup() {
  Serial.begin(9600);            // Start serial communication
}

void loop() {
  int raw = analogRead(fsrPin);                          // Read 10-bit ADC (0–1023)
  float Vout = raw * (Vcc / 1023.0);                     // Convert to volts
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.println(Vout, 3);                               // Show voltage with 3 decimals
  delay(100);                                            // Small delay for readability
}
