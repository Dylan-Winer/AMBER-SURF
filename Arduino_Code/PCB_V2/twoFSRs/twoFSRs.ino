const int fsr1Pin = A3;           // FSR 1 connected to analog input A0 
const float Vcc = 3.3;            // Teensy ADC reference voltage (typically 3.3 V)

void setup() {
  Serial.begin(9600);             // Start serial communication
}

void loop() {
  // Read analog values (10-bit resolution: 0–1023)
  int raw1 = analogRead(fsr1Pin);

  // Convert to voltage
  float Vout1 = raw1 * (Vcc / 1023.0);

  // Print readings
  Serial.print("FSR - Raw: ");
  Serial.print(raw1);
  Serial.print("\tVout: ");
  Serial.println(Vout1, 3);

  delay(100);  // 10 Hz sampling rate
}
