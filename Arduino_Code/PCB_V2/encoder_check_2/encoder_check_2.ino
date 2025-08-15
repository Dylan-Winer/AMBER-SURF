const int pwmPin = 37;
const float dutyMin = 1.0;
const float dutyMax = 99.0;
const float angleRange = 360.0;

float baselineOffset = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, INPUT);
  Serial.println("Starting 5-second baseline calibration...");

  unsigned long startTime = millis();
  int sampleCount = 0;
  float angleSum = 0;

  // Collect angle samples for 5 seconds
  while (millis() - startTime < 5000) {
    float angle = readRawAngle();
    if (!isnan(angle)) {
      angleSum += angle;
      sampleCount++;
    }
    delay(5); // Sample at ~200 Hz
  }

  if (sampleCount > 0) {
    baselineOffset = angleSum / sampleCount;
    Serial.print("Calibration complete. Baseline offset: ");
    Serial.print(baselineOffset, 2);
    Serial.println("°");
  } else {
    Serial.println("Calibration failed — no valid PWM signal.");
  }

  delay(1000);  // Give user time to read
}

void loop() {
  float angle = readRawAngle();

  if (!isnan(angle)) {
    // Apply offset correction
    float corrected = angle - baselineOffset;

    // Clamp negative or <2° values to 0
    if (corrected < 2.0f) corrected = 0.0f;

    Serial.print("Corrected Angle: ");
    Serial.print(corrected, 1);
    Serial.println("°");
  } else {
    Serial.println("No PWM signal detected.");
  }

  delay(10);  // ~100 Hz update
}

float readRawAngle() {
  unsigned long highTime = pulseIn(pwmPin, HIGH, 100000);
  unsigned long lowTime  = pulseIn(pwmPin, LOW, 100000);

  if (highTime == 0 && lowTime == 0) return NAN;

  unsigned long period = highTime + lowTime;
  float duty = (period > 0) ? (100.0f * highTime / period) : NAN;
  if (isnan(duty)) return NAN;

  // Clamp duty cycle to valid range
  duty = constrain(duty, dutyMin, dutyMax);

  // Map duty to angle
  float angle = (duty - dutyMin) / (dutyMax - dutyMin) * angleRange;
  return angle;
}
