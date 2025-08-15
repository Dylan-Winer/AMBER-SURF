const int fsrPin = A3;  
const float Vcc = 3.3;   // Teensy ADC reference voltage (typically 3.3 V)

void setup() {
  Serial.begin(9600);    // Start serial communication
}

void loop() {
  // Read analog values (10-bit resolution: 0–1023)
  int raw = analogRead(fsrPin);

  // Convert to voltage
  float Vout = raw * (Vcc / 1023.0);

  // Print readings
  Serial.print("FSR - Raw: ");
  Serial.print(raw);
  Serial.print("\tVout: ");
  Serial.println(Vout, 3);

  delay(200);  // sample rate
}
