const float Vcc = 3.3;  // Teensy ADC reference voltage

// FSR pin config
const int analogPins[7] = {A0, A3, A6, A9, A10, A11, A12};
const char* analogNames[7] = {"A0", "A3", "A6", "A9", "A10", "A11", "A12"};
const int ledPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 9};  // Added pins 7 and 8

// FSR calibration (Force = slope * Vout + intercept)
const float slope[7] = {
  233.257, 298.334, 243.102, 297.801, 602.777, 622.299, 435.190
};

const float intercept[7] = {
  -316.886, -403.587, -328.109, -403.681, -812.439, -841.463, -590.396
};

// Encoder config
const int pwmPin = 37;
const float dutyMin = 1.0;
const float dutyMax = 99.0;
const float angleRange = 360.0;
float baselineOffset = 0;

void setup() {
  Serial.begin(115200);

  // Set LED pins as output
  for (int i = 0; i < 9; i++) {  // Now initializing 9 pins
    pinMode(ledPins[i], OUTPUT);
  }

  // Set up encoder input pin
  pinMode(pwmPin, INPUT);

  // Encoder calibration (5 seconds)
  Serial.println("Starting 5-second encoder baseline calibration...");
  unsigned long startTime = millis();
  int sampleCount = 0;
  float angleSum = 0;

  while (millis() - startTime < 5000) {
    float angle = readRawAngle();
    if (!isnan(angle)) {
      angleSum += angle;
      sampleCount++;
    }
    delay(5);
  }

  if (sampleCount > 0) {
    baselineOffset = angleSum / sampleCount;
    Serial.print("Calibration complete. Baseline offset: ");
    Serial.print(baselineOffset, 2);
    Serial.println("°");
  } else {
    Serial.println("Calibration failed — no valid PWM signal.");
  }

  delay(1000);  // Pause to let user see message
}

void loop() {
  float forces[7];

  // === FSR READING + LED LOGIC ===
  for (int i = 0; i < 7; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);
    float Force = slope[i] * Vout + intercept[i];

    float threshold = (i >= 4) ? 4.5 : 2.5;
    if (Force < threshold) Force = 0.0;

    forces[i] = Force;

    // Print force only
    Serial.print(analogNames[i]);
    Serial.print(": ");
    Serial.print(Force, 1);
    Serial.print(" N\t");

    // LED control
    digitalWrite(ledPins[i], Force > threshold ? HIGH : LOW);
  }

  // === ENCODER ANGLE READING ===
  float angle = readRawAngle();
  float corrected = NAN;  // Declare outside so it's accessible after

  if (!isnan(angle)) {
    corrected = angle - baselineOffset;
    if (corrected < 2.0f) corrected = 0.0f;

    Serial.print("Angle: ");
    Serial.print(corrected, 1);
    Serial.println("°");
  } else {
    Serial.println("Angle: N/A (No PWM)");
  }

  // LED logic for angle
  digitalWrite(ledPins[7], (!isnan(corrected) && corrected > 30.0f) ? HIGH : LOW);
  digitalWrite(ledPins[8], (!isnan(corrected) && corrected > 60.0f) ? HIGH : LOW);

  delay(100);  // ~10 Hz loop
}

// Reads raw PWM angle
float readRawAngle() {
  unsigned long highTime = pulseIn(pwmPin, HIGH, 100000);
  unsigned long lowTime  = pulseIn(pwmPin, LOW, 100000);

  if (highTime == 0 && lowTime == 0) return NAN;

  unsigned long period = highTime + lowTime;
  float duty = (period > 0) ? (100.0f * highTime / period) : NAN;
  if (isnan(duty)) return NAN;

  duty = constrain(duty, dutyMin, dutyMax);
  float angle = (duty - dutyMin) / (dutyMax - dutyMin) * angleRange;
  return angle;
}
