#include <Wire.h>
#include <MadgwickAHRS.h>

// MPU6050 addresses
const uint8_t MPU1_ADDR = 0x68;
const uint8_t MPU2_ADDR = 0x69;

// Kalman gain for angular acceleration
const float kalmanK = 0.2;

// State variables
float prev_gx1 = 0, prev_gy1 = 0, prev_gz1 = 0;
float prev_gx2 = 0, prev_gy2 = 0, prev_gz2 = 0;
float angAccX1 = 0, angAccY1 = 0, angAccZ1 = 0;
float angAccX2 = 0, angAccY2 = 0, angAccZ2 = 0;

unsigned long lastTime1, lastTime2;

// Calibration offsets
float ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1;
float ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2;

// Madgwick filters
Madgwick filter1;
Madgwick filter2;

// Initialize MPU6050
void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

// Read 6-axis raw data from MPU6050
void readRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
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

// Basic calibration by averaging samples
void calibrate(uint8_t addr, float &ax_off, float &ay_off, float &az_off,
               float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(addr, ax, ay, az, gx, gy, gz);
    sum_ax += ax; sum_ay += ay; sum_az += az;
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delay(2);
  }
  ax_off = sum_ax / (float)samples;
  ay_off = sum_ay / (float)samples;
  az_off = sum_az / (float)samples - 16384.0f;
  gx_off = sum_gx / (float)samples;
  gy_off = sum_gy / (float)samples;
  gz_off = sum_gz / (float)samples;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();

  wakeMPU(MPU1_ADDR);
  wakeMPU(MPU2_ADDR);

  calibrate(MPU1_ADDR, ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1);
  calibrate(MPU2_ADDR, ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2);

  filter1.begin(10);        // actual loop rate is 10 Hz

  filter2.begin(10);

  lastTime1 = micros();
  lastTime2 = micros();
}

void loop() {
  // === Sensor 1 ===
  unsigned long now1 = micros();
  float dt1 = (now1 - lastTime1) * 1e-6f;
  if (dt1 <= 0.0f) return;

  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  readRaw(MPU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);
  float axn1 = (ax1 - ax_off1) / 16384.0f;
  float ayn1 = (ay1 - ay_off1) / 16384.0f;
  float azn1 = (az1 - az_off1) / 16384.0f;
  float gxn1 = (gx1 - gx_off1) / 131.0f;
  float gyn1 = (gy1 - gy_off1) / 131.0f;
  float gzn1 = (gz1 - gz_off1) / 131.0f;

  float rawAccX1 = (gxn1 - prev_gx1) / dt1;
  float rawAccY1 = (gyn1 - prev_gy1) / dt1;
  float rawAccZ1 = (gzn1 - prev_gz1) / dt1;
  angAccX1 = (1 - kalmanK) * angAccX1 + kalmanK * rawAccX1;
  angAccY1 = (1 - kalmanK) * angAccY1 + kalmanK * rawAccY1;
  angAccZ1 = (1 - kalmanK) * angAccZ1 + kalmanK * rawAccZ1;
  prev_gx1 = gxn1; prev_gy1 = gyn1; prev_gz1 = gzn1;

  filter1.updateIMU(gxn1 * DEG_TO_RAD, gyn1 * DEG_TO_RAD, gzn1 * DEG_TO_RAD, axn1, ayn1, azn1);
  lastTime1 = now1;

  // === Sensor 2 ===
  unsigned long now2 = micros();
  float dt2 = (now2 - lastTime2) * 1e-6f;
  if (dt2 <= 0.0f) return;

  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  readRaw(MPU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);
  float axn2 = (ax2 - ax_off2) / 16384.0f;
  float ayn2 = (ay2 - ay_off2) / 16384.0f;
  float azn2 = (az2 - az_off2) / 16384.0f;
  float gxn2 = (gx2 - gx_off2) / 131.0f;
  float gyn2 = (gy2 - gy_off2) / 131.0f;
  float gzn2 = (gz2 - gz_off2) / 131.0f;

  float rawAccX2 = (gxn2 - prev_gx2) / dt2;
  float rawAccY2 = (gyn2 - prev_gy2) / dt2;
  float rawAccZ2 = (gzn2 - prev_gz2) / dt2;
  angAccX2 = (1 - kalmanK) * angAccX2 + kalmanK * rawAccX2;
  angAccY2 = (1 - kalmanK) * angAccY2 + kalmanK * rawAccY2;
  angAccZ2 = (1 - kalmanK) * angAccZ2 + kalmanK * rawAccZ2;
  prev_gx2 = gxn2; prev_gy2 = gyn2; prev_gz2 = gzn2;

  filter2.updateIMU(gxn2 * DEG_TO_RAD, gyn2 * DEG_TO_RAD, gzn2 * DEG_TO_RAD, axn2, ayn2, azn2);
  lastTime2 = now2;

  // === OUTPUT ===
  float qw1, qx1, qy1, qz1;
  filter1.getQuaternion(&qw1, &qx1, &qy1, &qz1);

  Serial.print("Sensor1 Q = ");
  Serial.print(qw1); Serial.print(", ");
  Serial.print(qx1); Serial.print(", ");
  Serial.print(qy1); Serial.print(", ");
  Serial.println(qz1);

  Serial.print("Sensor1 AngAcc X,Y,Z = ");
  Serial.print(angAccX1, 2); Serial.print(", ");
  Serial.print(angAccY1, 2); Serial.print(", ");
  Serial.println(angAccZ1, 2);

  float qw2, qx2, qy2, qz2;
  filter2.getQuaternion(&qw2, &qx2, &qy2, &qz2);

  Serial.print("Sensor2 Q = ");
  Serial.print(qw2); Serial.print(", ");
  Serial.print(qx2); Serial.print(", ");
  Serial.print(qy2); Serial.print(", ");
  Serial.println(qz2);

  Serial.print("Sensor2 AngAcc X,Y,Z = ");
  Serial.print(angAccX2, 2); Serial.print(", ");
  Serial.print(angAccY2, 2); Serial.print(", ");
  Serial.println(angAccZ2, 2);

  delay(100); // loop rate ~10 Hz
}
