const int fsr1Pin = A1;           // FSR 1 connected to analog input A0
const int fsr2Pin = A0;           // FSR 2 connected to analog input A1
const int fsr3Pin = A2;           // FSR 3 connected to analog input A2
const int fsr4Pin = A3;           // FSR 4 connected to analog input A3
const float Vcc = 3.3;            // Teensy ADC reference voltage (typically 3.3 V)

void setup() {
  Serial.begin(9600);             // Start serial communication
}

void loop() {
  // Read analog values
  int raw1 = analogRead(fsr1Pin);
  int raw2 = analogRead(fsr2Pin);
  int raw3 = analogRead(fsr3Pin);
  int raw4 = analogRead(fsr4Pin);

  // Convert to voltage
  float Vout1 = raw1 * (Vcc / 1023.0);
  float Vout2 = raw2 * (Vcc / 1023.0);
  float Vout3 = raw3 * (Vcc / 1023.0);
  float Vout4 = raw4 * (Vcc / 1023.0);

  // Print readings
  Serial.print("FSR1 - Raw: ");
  Serial.print(raw1);
  Serial.print("\tVout: ");
  Serial.println(Vout1, 3);

  Serial.print("\tFSR2 - Raw: ");
  Serial.print(raw2);
  Serial.print("\tVout: ");
  Serial.println(Vout2, 3);

  Serial.print("\tFSR3 - Raw: ");
  Serial.print(raw3);
  Serial.print("\tVout: ");
  Serial.println(Vout3, 3);

  Serial.print("\tFSR4 - Raw: ");
  Serial.print(raw4);
  Serial.print("\tVout: ");
  Serial.println(Vout4, 3);
  
  delay(100);  // Delay for readability
}
