const int fsr1Pin = A0;           // FSR1: analog input
const int fsr2Pin = A7;           // FSR2: analog input
const int onboardLedPin = 13;     // Onboard LED (digital only)
const int externalLedPin = 11;    // External LED (PWM capable)
const int threshold = 300;        // Threshold for onboard LED

void setup() {
  Serial.begin(9600);
  pinMode(onboardLedPin, OUTPUT);
  pinMode(externalLedPin, OUTPUT);
}

void loop() {
  int fsr1Reading = analogRead(fsr1Pin);  // 0–1023
  int fsr2Reading = analogRead(fsr2Pin);  // 0–1023

  Serial.print("FSR1: ");
  Serial.print(fsr1Reading);
  Serial.print("  FSR2: ");
  Serial.println(fsr2Reading);

  // FSR1 → onboard LED (digital ON/OFF based on pressure)
  if (fsr1Reading > threshold) {
    digitalWrite(onboardLedPin, HIGH);
  } else {
    digitalWrite(onboardLedPin, LOW);
  }

  // FSR2 → external LED (PWM brightness)
  int brightness = map(fsr2Reading, 0, 1023, 0, 255);
  brightness = constrain(brightness, 0, 255);
  analogWrite(externalLedPin, brightness);

  delay(50);
}
