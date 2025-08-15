#include <Wire.h>

const float Vcc = 3.3;

// === FSRs ===
const int analogPins[7] = {A0, A3, A6, A9, A10, A11, A12};
const char* analogNames[7] = {"A0", "A3", "A6", "A9", "A10", "A11", "A12"};
const int ledPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 9};
const float slope[7] = {233.257, 298.334, 243.102, 297.801, 602.777, 622.299, 435.190};
const float intercept[7] = {-316.886, -403.587, -328.109, -403.681, -812.439, -841.463, -590.396};

// === Encoder ===
const int pwmPin = 37;
const float dutyMin = 1.0, dutyMax = 99.0, angleRange = 360.0;
float baselineOffset = 0;

// === IMUs ===
const uint8_t MPU1_ADDR = 0x68;
const uint8_t MPU2_ADDR = 0x69;
float ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1;
float ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2;
float q0_1=1, q1_1=0, q2_1=0, q3_1=0;
float q0_2=1, q1_2=0, q2_2=0, q3_2=0;
float prev_gx1, prev_gy1, prev_gz1, prev_gx2, prev_gy2, prev_gz2;
unsigned long lastTime;
const float accWeight = 0.02f;

void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); delay(100);
}

void readRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)14);
  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // skip temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

void calibrateSensor(uint8_t addr,
                     float &ax_off, float &ay_off, float &az_off,
                     float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_ax=0, sum_ay=0, sum_az=0, sum_gx=0, sum_gy=0, sum_gz=0;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(addr, ax, ay, az, gx, gy, gz);
    sum_ax += ax; sum_ay += ay; sum_az += az;
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delay(3);
  }
  ax_off = sum_ax / (float)samples;
  ay_off = sum_ay / (float)samples;
  az_off = (sum_az / (float)samples) - 16384.0f;
  gx_off = sum_gx / (float)samples;
  gy_off = sum_gy / (float)samples;
  gz_off = sum_gz / (float)samples;
}

void setup() {
  Serial.begin(115200);
  Wire.setSDA(18); Wire.setSCL(19); Wire.begin();

  for (int i = 0; i < 9; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(pwmPin, INPUT);

  wakeMPU(MPU1_ADDR); wakeMPU(MPU2_ADDR);
  calibrateSensor(MPU1_ADDR, ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1);
  calibrateSensor(MPU2_ADDR, ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2);

  int16_t ax, ay, az, gx, gy, gz;
  readRaw(MPU1_ADDR, ax, ay, az, gx, gy, gz);
  prev_gx1 = (gx - gx_off1) / 131.0f;
  prev_gy1 = (gy - gy_off1) / 131.0f;
  prev_gz1 = (gz - gz_off1) / 131.0f;
  readRaw(MPU2_ADDR, ax, ay, az, gx, gy, gz);
  prev_gx2 = (gx - gx_off2) / 131.0f;
  prev_gy2 = (gy - gy_off2) / 131.0f;
  prev_gz2 = (gz - gz_off2) / 131.0f;

  // Encoder calibration
  Serial.println("Starting 5-second encoder baseline calibration...");
  unsigned long startTime = millis();
  int sampleCount = 0; float angleSum = 0;
  while (millis() - startTime < 5000) {
    float angle = readRawAngle();
    if (!isnan(angle)) { angleSum += angle; sampleCount++; }
    delay(5);
  }
  if (sampleCount > 0) {
    baselineOffset = angleSum / sampleCount;
    Serial.print("Baseline offset: "); Serial.print(baselineOffset, 2); Serial.println("°");
  } else {
    Serial.println("Encoder calibration failed.");
  }

  lastTime = micros();
  Serial.println("System ready.\n");
}

void loop() {
  float dt = (micros() - lastTime) * 1e-6f;
  if (dt <= 0) dt = 1e-6f;
  lastTime = micros();

  float forces[7];
  for (int i = 0; i < 7; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0);
    float Force = slope[i] * Vout + intercept[i];
    float threshold = (i >= 4) ? 4.5 : 2.5;
    if (Force < threshold) Force = 0.0;
    forces[i] = Force;

    digitalWrite(ledPins[i], Force > threshold ? HIGH : LOW);
  }

  float angle = readRawAngle();
  float corrected = isnan(angle) ? NAN : angle - baselineOffset;
  if (!isnan(corrected) && corrected < 2.0f) corrected = 0.0f;
  digitalWrite(ledPins[7], (!isnan(corrected) && corrected > 30.0f) ? HIGH : LOW);
  digitalWrite(ledPins[8], (!isnan(corrected) && corrected > 60.0f) ? HIGH : LOW);

  // Output forces
  for (int i = 0; i < 7; i++) {
    Serial.print(analogNames[i]); Serial.print(": ");
    Serial.print(forces[i], 1); Serial.print(" N\t");
  }

  Serial.print("Angle: ");
  Serial.print(!isnan(corrected) ? corrected : 0.0, 1);
  Serial.println("°");

  // === Process MPU 1 and 2 ===
  // Each IMU block should repeat the quaternion + angAcc logic.
  // For brevity, here we call a helper twice:
  processMPU(MPU1_ADDR, q0_1, q1_1, q2_1, q3_1, prev_gx1, prev_gy1, prev_gz1,
             ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1,
             "Sensor1");

  processMPU(MPU2_ADDR, q0_2, q1_2, q2_2, q3_2, prev_gx2, prev_gy2, prev_gz2,
             ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2,
             "Sensor2");

  Serial.println();
  delay(100);  // 10 Hz loop
}

float readRawAngle() {
  unsigned long highTime = pulseIn(pwmPin, HIGH, 100000);
  unsigned long lowTime  = pulseIn(pwmPin, LOW, 100000);
  if (highTime == 0 && lowTime == 0) return NAN;
  float duty = 100.0f * highTime / (highTime + lowTime);
  duty = constrain(duty, dutyMin, dutyMax);
  return (duty - dutyMin) / (dutyMax - dutyMin) * angleRange;
}

void processMPU(uint8_t addr,
                float &q0, float &q1, float &q2, float &q3,
                float &prev_gx, float &prev_gy, float &prev_gz,
                float ax_off, float ay_off, float az_off,
                float gx_off, float gy_off, float gz_off,
                const char* label) {
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  readRaw(addr, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);
  float axn = (raw_ax - ax_off) / 16384.0f;
  float ayn = (raw_ay - ay_off) / 16384.0f;
  float azn = (raw_az - az_off) / 16384.0f;
  float gxn = (raw_gx - gx_off) / 131.0f;
  float gyn = (raw_gy - gy_off) / 131.0f;
  float gzn = (raw_gz - gz_off) / 131.0f;

  float angAccX = (gxn - prev_gx) / (1e-6f + (micros() - lastTime) * 1e-6f);
  float angAccY = (gyn - prev_gy) / (1e-6f + (micros() - lastTime) * 1e-6f);
  float angAccZ = (gzn - prev_gz) / (1e-6f + (micros() - lastTime) * 1e-6f);
  prev_gx = gxn; prev_gy = gyn; prev_gz = gzn;

  float gx_r = gxn * DEG_TO_RAD, gy_r = gyn * DEG_TO_RAD, gz_r = gzn * DEG_TO_RAD;
  float qDot0 = -0.5f*(q1*gx_r + q2*gy_r + q3*gz_r);
  float qDot1 =  0.5f*(q0*gx_r + q2*gz_r - q3*gy_r);
  float qDot2 =  0.5f*(q0*gy_r - q1*gz_r + q3*gx_r);
  float qDot3 =  0.5f*(q0*gz_r + q1*gy_r - q2*gx_r);
  q0 += qDot0 * 0.01f; q1 += qDot1 * 0.01f; q2 += qDot2 * 0.01f; q3 += qDot3 * 0.01f;
  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;

  float roll  = atan2f(ayn, azn);
  float pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));
  float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
  float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
  float q_acc0 = cp*cr, q_acc1 = cp*sr, q_acc2 = sp*cr, q_acc3 = -sp*sr;
  q0 = (1 - accWeight)*q0 + accWeight*q_acc0;
  q1 = (1 - accWeight)*q1 + accWeight*q_acc1;
  q2 = (1 - accWeight)*q2 + accWeight*q_acc2;
  q3 = (1 - accWeight)*q3 + accWeight*q_acc3;
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;

  Serial.print(label); Serial.print(" Q = ");
  Serial.print(q0,6); Serial.print(","); Serial.print(q1,6); Serial.print(",");
  Serial.print(q2,6); Serial.print(","); Serial.println(q3,6);

  Serial.print(label); Serial.print(" AngAcc X,Y,Z = ");
  Serial.print(angAccX,2); Serial.print(","); Serial.print(angAccY,2); Serial.print(",");
  Serial.println(angAccZ,2);
}

