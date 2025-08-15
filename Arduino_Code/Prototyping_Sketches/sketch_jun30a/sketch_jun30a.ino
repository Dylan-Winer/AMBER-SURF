const int pwmPin = 9;

void setup() {
  analogWriteFrequency(pwmPin, 5000);
  analogWriteResolution(8); // 8-bit PWM
  analogWrite(pwmPin, 51);  // ~20% of 255 = ~0.66V
}

void loop() {
  // Nothing needed
}
