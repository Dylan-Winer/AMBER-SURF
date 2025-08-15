const int pwmPin = 37;  // Teensy 4.1 pin 37 for PWM input
const float dutyMin = 1.0;    // MAE4 min duty cycle (%)
const float dutyMax = 99.0;   // MAE4 max duty cycle (%)
const float angleRange = 360.0; // Total rotation range (degrees)

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, INPUT);
  Serial.println("Teensy PWM Angle Reader Initialized");
}

void loop() {
  unsigned long highTime = pulseIn(pwmPin, HIGH, 100000); // 100 ms timeout
  unsigned long lowTime  = pulseIn(pwmPin, LOW, 100000);

  if (highTime == 0 && lowTime == 0) {
    Serial.println("No PWM signal detected — check power supply & wiring");
  } else {
    unsigned long period = highTime + lowTime;
    float duty = (period > 0) ? (100.0f * highTime / period) : 0;
    float freq = (period > 0) ? (1e6f / period) : 0;

    // Clamp duty cycle to [1%, 99%]
    duty = constrain(duty, dutyMin, dutyMax);

    // Convert to angle: map duty cycle to 0–360 degrees
    float angle = (duty - dutyMin) / (dutyMax - dutyMin) * angleRange;

    // Output
    Serial.print("Duty Cycle: ");
    Serial.print(duty, 2);
    Serial.print("% → Angle: ");
    Serial.print(angle, 1);
    Serial.println("°");
  }

  delay(200);  // ~5 Hz update rate
}
