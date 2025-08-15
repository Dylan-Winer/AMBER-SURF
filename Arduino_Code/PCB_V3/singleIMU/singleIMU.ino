// Single-IMU (MPU-6050) with Madgwick + EMA-smoothed angular acceleration
// FIX: calibrate GYRO ONLY (do NOT subtract 1g from accel). Feed accel with gravity intact.

#include <Wire.h>
#include <MadgwickAHRS.h>

const uint8_t MPU_ADDR = 0x68;   // AD0 = GND
Madgwick filter;

float gx_off = 0, gy_off = 0, gz_off = 0;   // gyro bias (deg/s) only

float prev_gx = 0, prev_gy = 0, prev_gz = 0; // previous gyro (deg/s)
float angAccX = 0, angAccY = 0, angAccZ = 0; // EMA-smoothed angular accel (deg/s^2)
unsigned long lastTime = 0;

const float kalmanK = 0.2f;      // EMA factor for angular accel smoothing

// --- IMU helpers ---
void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);    // PWR_MGMT_1
  Wire.write(0x00);    // Wake from sleep
  Wire.endTransmission();
  delay(100);
}

void readRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az,
                        int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)14);
  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

// GYRO-ONLY calibration: estimate gyro biases (deg/s). DO NOT touch accelerometer.
void calibrateGyro(uint8_t addr, float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(addr, ax, ay, az, gx, gy, gz);
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delay(2);
  }
  // Convert to deg/s for the bias terms (±250 dps => 131 LSB/(deg/s))
  gx_off = (sum_gx / (float)samples) / 131.0f;
  gy_off = (sum_gy / (float)samples) / 131.0f;
  gz_off = (sum_gz / (float)samples) / 131.0f;
}

// --- Arduino setup/loop ---
void setup() {
  Serial.begin(115200);
  Wire.setSDA(18); Wire.setSCL(19); Wire.begin();

  wakeMPU(MPU_ADDR);
  calibrateGyro(MPU_ADDR, gx_off, gy_off, gz_off);   // <-- only gyro!

  filter.begin(10);       // match ~10 Hz loop below
  lastTime = micros();

  Serial.println("IMU-only system ready (gyro-cal only).\n");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  if (dt > 0.0f) {
    // Read raw IMU
    int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
    readRaw(MPU_ADDR, ax_i, ay_i, az_i, gx_i, gy_i, gz_i);

    // Accel in g (±2g => 16384 LSB/g). KEEP GRAVITY.
    float ax = ax_i / 16384.0f;
    float ay = ay_i / 16384.0f;
    float az = az_i / 16384.0f;

    // Gyro in deg/s minus bias
    float gxd = (gx_i / 131.0f) - gx_off;
    float gyd = (gy_i / 131.0f) - gy_off;
    float gzd = (gz_i / 131.0f) - gz_off;

    // Angular acceleration via EMA on derivative of gyro (deg/s^2)
    float rawAccX = (gxd - prev_gx) / dt;
    float rawAccY = (gyd - prev_gy) / dt;
    float rawAccZ = (gzd - prev_gz) / dt;
    angAccX = (1 - kalmanK) * angAccX + kalmanK * rawAccX;
    angAccY = (1 - kalmanK) * angAccY + kalmanK * rawAccY;
    angAccZ = (1 - kalmanK) * angAccZ + kalmanK * rawAccZ;
    prev_gx = gxd; prev_gy = gyd; prev_gz = gzd;

    // Madgwick update: gyro in rad/s, accel in g
    filter.updateIMU(gxd * DEG_TO_RAD, gyd * DEG_TO_RAD, gzd * DEG_TO_RAD,
                     ax, ay, az);

    lastTime = now;
  }

  // --- Serial Output ---
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Q = ");
  Serial.print(qw); Serial.print(", ");
  Serial.print(qx); Serial.print(", ");
  Serial.print(qy); Serial.print(", ");
  Serial.println(qz);

  Serial.print("AngAcc X,Y,Z = ");
  Serial.print(angAccX, 2); Serial.print(", ");
  Serial.print(angAccY, 2); Serial.print(", ");
  Serial.println(angAccZ, 2);

  Serial.println();
  delay(100);  // ~10 Hz loop
}
