#include <Wire.h>

// I2C address for MPU-6050 with AD0 = 3.3V
const uint8_t MPU_ADDR = 0x69;

// Calibration offsets
float ax_off, ay_off, az_off, gx_off, gy_off, gz_off;

// Quaternion state
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

// Previous normalized gyro readings for angular acceleration
float prev_gx = 0, prev_gy = 0, prev_gz = 0;

// Timing
unsigned long lastTime;
const float accWeight = 0.02f;

// Wake up MPU by clearing sleep bit
void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

// Read raw accelerometer + gyro from given address
void readRaw(uint8_t addr, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
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

// Calibrate MPU
void calibrateSensor(uint8_t addr, float &ax_off, float &ay_off, float &az_off, float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  Serial.print("Calibrating 0x"); Serial.println(addr, HEX);
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
  while (!Serial) {}

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();

  wakeMPU(MPU_ADDR);
  calibrateSensor(MPU_ADDR, ax_off, ay_off, az_off, gx_off, gy_off, gz_off);

  int16_t ax, ay, az, gx, gy, gz;
  readRaw(MPU_ADDR, ax, ay, az, gx, gy, gz);
  prev_gx = (gx - gx_off) / 131.0f;
  prev_gy = (gy - gy_off) / 131.0f;
  prev_gz = (gz - gz_off) / 131.0f;

  lastTime = micros();
  Serial.println("Calibration complete. Starting loop…\n");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  if (dt <= 0) dt = 1e-6f;

  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  readRaw(MPU_ADDR, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);
  float axn = (raw_ax - ax_off) / 16384.0f;
  float ayn = (raw_ay - ay_off) / 16384.0f;
  float azn = (raw_az - az_off) / 16384.0f;
  float gxn = (raw_gx - gx_off) / 131.0f;
  float gyn = (raw_gy - gy_off) / 131.0f;
  float gzn = (raw_gz - gz_off) / 131.0f;

  float angAccX = (gxn - prev_gx) / dt;
  float angAccY = (gyn - prev_gy) / dt;
  float angAccZ = (gzn - prev_gz) / dt;
  prev_gx = gxn; prev_gy = gyn; prev_gz = gzn;

  float gx_r = gxn*(PI/180.0f), gy_r = gyn*(PI/180.0f), gz_r = gzn*(PI/180.0f);
  float qDot0 = -0.5f*(q1*gx_r + q2*gy_r + q3*gz_r);
  float qDot1 =  0.5f*(q0*gx_r + q2*gz_r - q3*gy_r);
  float qDot2 =  0.5f*(q0*gy_r - q1*gz_r + q3*gx_r);
  float qDot3 =  0.5f*(q0*gz_r + q1*gy_r - q2*gx_r);
  q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt;
  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;

  float roll_acc  = atan2f(ayn, azn);
  float pitch_acc = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));
  float cr = cosf(roll_acc*0.5f), sr = sinf(roll_acc*0.5f);
  float cp = cosf(pitch_acc*0.5f), sp = sinf(pitch_acc*0.5f);
  float q_acc0 = cp*cr, q_acc1 = cp*sr, q_acc2 = sp*cr, q_acc3 = -sp*sr;
  q0 = (1-accWeight)*q0 + accWeight*q_acc0;
  q1 = (1-accWeight)*q1 + accWeight*q_acc1;
  q2 = (1-accWeight)*q2 + accWeight*q_acc2;
  q3 = (1-accWeight)*q3 + accWeight*q_acc3;
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;

  Serial.print("Q = ");
  Serial.print(q0,6); Serial.print(',');
  Serial.print(q1,6); Serial.print(',');
  Serial.print(q2,6); Serial.print(',');
  Serial.println(q3,6);

  Serial.print("AngAcc X,Y,Z = ");
  Serial.print(angAccX,2); Serial.print(',');
  Serial.print(angAccY,2); Serial.print(',');
  Serial.println(angAccZ,2);

  delay(500);  // 2 Hz sampling
}
