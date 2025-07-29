const int fsrPin = A0;       // FSR voltage divider output
const int onboardLedPin = 13;  // Onboard LED on Teensy
const int externalLedPin = 11; // External LED connected to pin 11
const int threshold = 300;     // Adjust based on pressure level

void setup() {
  Serial.begin(9600);
  pinMode(onboardLedPin, OUTPUT);
  pinMode(externalLedPin, OUTPUT);
}

void loop() {
  int fsrReading = analogRead(fsrPin);  // Read pressure
  Serial.println(fsrReading);           // Print to Serial Monitor

  if (fsrReading > threshold) {
    digitalWrite(onboardLedPin, HIGH);
    digitalWrite(externalLedPin, HIGH);
  } else {
    digitalWrite(onboardLedPin, LOW);
    digitalWrite(externalLedPin, LOW);
  }

  delay(100);  // Delay for readability
}
