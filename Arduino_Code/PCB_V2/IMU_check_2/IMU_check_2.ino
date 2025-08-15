#include <Wire.h>

// MPU Addresses
const uint8_t MPU1_ADDR = 0x68;
const uint8_t MPU2_ADDR = 0x69;

// Calibration offsets
float ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1;
float ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2;

// Quaternion state
float q0_1 = 1, q1_1 = 0, q2_1 = 0, q3_1 = 0;
float q0_2 = 1, q1_2 = 0, q2_2 = 0, q3_2 = 0;

// Previous gyro readings for angular acceleration
float prev_gx1 = 0, prev_gy1 = 0, prev_gz1 = 0;
float prev_gx2 = 0, prev_gy2 = 0, prev_gz2 = 0;

// Time trackers
unsigned long lastTime1, lastTime2;

const float accWeight = 0.02f;

void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

void readRaw(uint8_t addr,
             int16_t &ax, int16_t &ay, int16_t &az,
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

void calibrateSensor(uint8_t addr,
                     float &ax_off, float &ay_off, float &az_off,
                     float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_ax=0, sum_ay=0, sum_az=0;
  long sum_gx=0, sum_gy=0, sum_gz=0;
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

  wakeMPU(MPU1_ADDR);
  wakeMPU(MPU2_ADDR);

  calibrateSensor(MPU1_ADDR, ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1);
  calibrateSensor(MPU2_ADDR, ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2);

  // Prime previous gyro readings
  {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(MPU1_ADDR, ax, ay, az, gx, gy, gz);
    prev_gx1 = (gx - gx_off1) / 131.0f;
    prev_gy1 = (gy - gy_off1) / 131.0f;
    prev_gz1 = (gz - gz_off1) / 131.0f;
    readRaw(MPU2_ADDR, ax, ay, az, gx, gy, gz);
    prev_gx2 = (gx - gx_off2) / 131.0f;
    prev_gy2 = (gy - gy_off2) / 131.0f;
    prev_gz2 = (gz - gz_off2) / 131.0f;
  }

  lastTime1 = micros();
  lastTime2 = micros();

  Serial.println("Calibration complete. Starting loop…\n");
}

void loop() {
  // === SENSOR 1 ===
  unsigned long now1 = micros();
  float dt1 = (now1 - lastTime1) * 1e-6f;
  lastTime1 = now1;
  if (dt1 <= 0) dt1 = 1e-6f;

  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  readRaw(MPU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);
  float axn1 = (ax1 - ax_off1) / 16384.0f;
  float ayn1 = (ay1 - ay_off1) / 16384.0f;
  float azn1 = (az1 - az_off1) / 16384.0f;
  float gxn1 = (gx1 - gx_off1) / 131.0f;
  float gyn1 = (gy1 - gy_off1) / 131.0f;
  float gzn1 = (gz1 - gz_off1) / 131.0f;

  float angAccX1 = (gxn1 - prev_gx1) / dt1;
  float angAccY1 = (gyn1 - prev_gy1) / dt1;
  float angAccZ1 = (gzn1 - prev_gz1) / dt1;
  prev_gx1 = gxn1; prev_gy1 = gyn1; prev_gz1 = gzn1;

  float gx_r1 = gxn1 * DEG_TO_RAD, gy_r1 = gyn1 * DEG_TO_RAD, gz_r1 = gzn1 * DEG_TO_RAD;
  float qDot0_1 = -0.5f*(q1_1*gx_r1 + q2_1*gy_r1 + q3_1*gz_r1);
  float qDot1_1 =  0.5f*(q0_1*gx_r1 + q2_1*gz_r1 - q3_1*gy_r1);
  float qDot2_1 =  0.5f*(q0_1*gy_r1 - q1_1*gz_r1 + q3_1*gx_r1);
  float qDot3_1 =  0.5f*(q0_1*gz_r1 + q1_1*gy_r1 - q2_1*gx_r1);
  q0_1 += qDot0_1*dt1; q1_1 += qDot1_1*dt1; q2_1 += qDot2_1*dt1; q3_1 += qDot3_1*dt1;
  float norm1 = sqrtf(q0_1*q0_1 + q1_1*q1_1 + q2_1*q2_1 + q3_1*q3_1);
  q0_1/=norm1; q1_1/=norm1; q2_1/=norm1; q3_1/=norm1;

  float roll_acc1  = atan2f(ayn1, azn1);
  float pitch_acc1 = atan2f(-axn1, sqrtf(ayn1*ayn1 + azn1*azn1));
  float cr1 = cosf(roll_acc1*0.5f), sr1 = sinf(roll_acc1*0.5f);
  float cp1 = cosf(pitch_acc1*0.5f), sp1 = sinf(pitch_acc1*0.5f);
  float q_acc0_1 = cp1*cr1, q_acc1_1 = cp1*sr1, q_acc2_1 = sp1*cr1, q_acc3_1 = -sp1*sr1;
  q0_1 = (1-accWeight)*q0_1 + accWeight*q_acc0_1;
  q1_1 = (1-accWeight)*q1_1 + accWeight*q_acc1_1;
  q2_1 = (1-accWeight)*q2_1 + accWeight*q_acc2_1;
  q3_1 = (1-accWeight)*q3_1 + accWeight*q_acc3_1;
  norm1 = sqrtf(q0_1*q0_1 + q1_1*q1_1 + q2_1*q2_1 + q3_1*q3_1);
  q0_1/=norm1; q1_1/=norm1; q2_1/=norm1; q3_1/=norm1;

  // === SENSOR 2 ===
  unsigned long now2 = micros();
  float dt2 = (now2 - lastTime2) * 1e-6f;
  lastTime2 = now2;
  if (dt2 <= 0) dt2 = 1e-6f;

  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  readRaw(MPU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);
  float axn2 = (ax2 - ax_off2) / 16384.0f;
  float ayn2 = (ay2 - ay_off2) / 16384.0f;
  float azn2 = (az2 - az_off2) / 16384.0f;
  float gxn2 = (gx2 - gx_off2) / 131.0f;
  float gyn2 = (gy2 - gy_off2) / 131.0f;
  float gzn2 = (gz2 - gz_off2) / 131.0f;

  float angAccX2 = (gxn2 - prev_gx2) / dt2;
  float angAccY2 = (gyn2 - prev_gy2) / dt2;
  float angAccZ2 = (gzn2 - prev_gz2) / dt2;
  prev_gx2 = gxn2; prev_gy2 = gyn2; prev_gz2 = gzn2;

  float gx_r2 = gxn2 * DEG_TO_RAD, gy_r2 = gyn2 * DEG_TO_RAD, gz_r2 = gzn2 * DEG_TO_RAD;
  float qDot0_2 = -0.5f*(q1_2*gx_r2 + q2_2*gy_r2 + q3_2*gz_r2);
  float qDot1_2 =  0.5f*(q0_2*gx_r2 + q2_2*gz_r2 - q3_2*gy_r2);
  float qDot2_2 =  0.5f*(q0_2*gy_r2 - q1_2*gz_r2 + q3_2*gx_r2);
  float qDot3_2 =  0.5f*(q0_2*gz_r2 + q1_2*gy_r2 - q2_2*gx_r2);
  q0_2 += qDot0_2*dt2; q1_2 += qDot1_2*dt2; q2_2 += qDot2_2*dt2; q3_2 += qDot3_2*dt2;
  float norm2 = sqrtf(q0_2*q0_2 + q1_2*q1_2 + q2_2*q2_2 + q3_2*q3_2);
  q0_2/=norm2; q1_2/=norm2; q2_2/=norm2; q3_2/=norm2;

  float roll_acc2  = atan2f(ayn2, azn2);
  float pitch_acc2 = atan2f(-axn2, sqrtf(ayn2*ayn2 + azn2*azn2));
  float cr2 = cosf(roll_acc2*0.5f), sr2 = sinf(roll_acc2*0.5f);
  float cp2 = cosf(pitch_acc2*0.5f), sp2 = sinf(pitch_acc2*0.5f);
  float q_acc0_2 = cp2*cr2, q_acc1_2 = cp2*sr2, q_acc2_2 = sp2*cr2, q_acc3_2 = -sp2*sr2;
  q0_2 = (1-accWeight)*q0_2 + accWeight*q_acc0_2;
  q1_2 = (1-accWeight)*q1_2 + accWeight*q_acc1_2;
  q2_2 = (1-accWeight)*q2_2 + accWeight*q_acc2_2;
  q3_2 = (1-accWeight)*q3_2 + accWeight*q_acc3_2;
  norm2 = sqrtf(q0_2*q0_2 + q1_2*q1_2 + q2_2*q2_2 + q3_2*q3_2);
  q0_2/=norm2; q1_2/=norm2; q2_2/=norm2; q3_2/=norm2;

  // === OUTPUT ===
  Serial.print("Sensor1 Q = ");
  Serial.print(q0_1,6); Serial.print(',');
  Serial.print(q1_1,6); Serial.print(',');
  Serial.print(q2_1,6); Serial.print(',');
  Serial.println(q3_1,6);

  Serial.print("Sensor1 AngAcc X,Y,Z = ");
  Serial.print(angAccX1,2); Serial.print(',');
  Serial.print(angAccY1,2); Serial.print(',');
  Serial.println(angAccZ1,2);

  Serial.print("Sensor2 Q = ");
  Serial.print(q0_2,6); Serial.print(',');
  Serial.print(q1_2,6); Serial.print(',');
  Serial.print(q2_2,6); Serial.print(',');
  Serial.println(q3_2,6);

  Serial.print("Sensor2 AngAcc X,Y,Z = ");
  Serial.print(angAccX2,2); Serial.print(',');
  Serial.print(angAccY2,2); Serial.print(',');
  Serial.println(angAccZ2,2);

  delay(100);  // 10 Hz
}
