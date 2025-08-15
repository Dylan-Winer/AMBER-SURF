const int fsr1Pin = A0;           // FSR1: analog input
const int fsr2Pin = A7;           // FSR2: analog input
const int led1Pin = A6;     // Blue LED
const int led2Pin = A1;    // External LED (PWM capable)
const int threshold = 50;        // Threshold for onboard LED

void setup() {
  Serial.begin(9600);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
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
    digitalWrite(led1Pin, HIGH);
  } else {
    digitalWrite(led1Pin, LOW);
  }

  if (fsr2Reading > threshold) {
    digitalWrite(led2Pin, HIGH);
  } else {
    digitalWrite(led2Pin, LOW);
  }


  delay(50);
}
