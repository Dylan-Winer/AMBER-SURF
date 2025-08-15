#include <Wire.h>
#include <MadgwickAHRS.h>

const uint8_t MPU_ADDR = 0x68;  // AD0 = GND

Madgwick filter;
const float kalmanK = 0.2;

float ax_off, ay_off, az_off, gx_off, gy_off, gz_off;
float prev_gx = 0, prev_gy = 0, prev_gz = 0;
float angAccX = 0, angAccY = 0, angAccZ = 0;
unsigned long lastTime = 0;

void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake from sleep
  Wire.endTransmission();
  delay(100);
}

void readRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);  // Starting register
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)14);
  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // Skip temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

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
  az_off = sum_az / (float)samples - 16384.0f;  // Remove 1g
  gx_off = sum_gx / (float)samples;
  gy_off = sum_gy / (float)samples;
  gz_off = sum_gz / (float)samples;
}

void setup() {
  Serial.begin(115200);
  Wire.setSDA(18);  // SDA on Teensy 4.1
  Wire.setSCL(19);  // SCL on Teensy 4.1
  Wire.begin();

  wakeMPU(MPU_ADDR);
  calibrate(MPU_ADDR, ax_off, ay_off, az_off, gx_off, gy_off, gz_off);

  filter.begin(10);  // ~10 Hz expected loop rate
  lastTime = micros();

  Serial.println("Single IMU initialized.\n");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  if (dt <= 0.0f) dt = 1e-6f;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  readRaw(MPU_ADDR, ax, ay, az, gx, gy, gz);

  float axn = (ax - ax_off) / 16384.0f;
  float ayn = (ay - ay_off) / 16384.0f;
  float azn = (az - az_off) / 16384.0f;
  float gxn = (gx - gx_off) / 131.0f;
  float gyn = (gy - gy_off) / 131.0f;
  float gzn = (gz - gz_off) / 131.0f;

  // Angular acceleration (smoothed)
  float rawAccX = (gxn - prev_gx) / dt;
  float rawAccY = (gyn - prev_gy) / dt;
  float rawAccZ = (gzn - prev_gz) / dt;
  angAccX = (1 - kalmanK) * angAccX + kalmanK * rawAccX;
  angAccY = (1 - kalmanK) * angAccY + kalmanK * rawAccY;
  angAccZ = (1 - kalmanK) * angAccZ + kalmanK * rawAccZ;
  prev_gx = gxn; prev_gy = gyn; prev_gz = gzn;

  // Update quaternion
  filter.updateIMU(gxn * DEG_TO_RAD, gyn * DEG_TO_RAD, gzn * DEG_TO_RAD, axn, ayn, azn);

  // Output
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Q = ");
  Serial.print(qw, 6); Serial.print(", ");
  Serial.print(qx, 6); Serial.print(", ");
  Serial.print(qy, 6); Serial.print(", ");
  Serial.println(qz, 6);

  Serial.print("AngAcc X,Y,Z = ");
  Serial.print(angAccX, 2); Serial.print(", ");
  Serial.print(angAccY, 2); Serial.print(", ");
  Serial.println(angAccZ, 2);

  Serial.println();
  delay(100);  // ~10 Hz loop
}
