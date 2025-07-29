#include <Wire.h>

// I2C addresses for the two MPU-6050s (AD0=GND → 0x68; AD0=3V3 → 0x69)
const uint8_t MPU1_ADDR = 0x68;
const uint8_t MPU2_ADDR = 0x69;

// Calibration offsets
float ax_off1, ay_off1, az_off1, gx_off1, gy_off1, gz_off1;
float ax_off2, ay_off2, az_off2, gx_off2, gy_off2, gz_off2;

// Quaternion state: sensor 1
float q0_1 = 1, q1_1 = 0, q2_1 = 0, q3_1 = 0;
// Quaternion state: sensor 2
float q0_2 = 1, q1_2 = 0, q2_2 = 0, q3_2 = 0;

// Previous normalized gyro readings for angular acceleration
float prev_gx1 = 0, prev_gy1 = 0, prev_gz1 = 0;
float prev_gx2 = 0, prev_gy2 = 0, prev_gz2 = 0;

// Timing
unsigned long lastTime;
const float accWeight = 0.02f;  // accelerometer “trust” in complementary filter

// Wake up MPU by clearing sleep bit
void wakeMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

// Read raw accelerometer + gyro from given address
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

// Calibrate one MPU, filling offsets
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
  az_off = (sum_az / (float)samples) - 16384.0f;  // remove 1 g
  gx_off = sum_gx / (float)samples;
  gy_off = sum_gy / (float)samples;
  gz_off = sum_gz / (float)samples;
  Serial.print("Offsets: ");
  Serial.print(ax_off,1); Serial.print(", ");
  Serial.print(ay_off,1); Serial.print(", ");
  Serial.print(az_off,1); Serial.print(", ");
  Serial.print(gx_off,1); Serial.print(", ");
  Serial.print(gy_off,1); Serial.print(", ");
  Serial.println(gz_off,1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Route I²C to pins 18 (SDA) / 19 (SCL)
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();

  // Wake both MPUs
  wakeMPU(MPU1_ADDR);
  wakeMPU(MPU2_ADDR);

  // Calibrate both
  calibrateSensor(MPU1_ADDR,
                  ax_off1, ay_off1, az_off1,
                  gx_off1, gy_off1, gz_off1);
  calibrateSensor(MPU2_ADDR,
                  ax_off2, ay_off2, az_off2,
                  gx_off2, gy_off2, gz_off2);

  // Prime previous gyro readings to avoid dt=0 spike
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

  lastTime = micros();
  Serial.println("Calibration complete. Starting loop…\n");
}

void loop() {
  // Compute dt and clamp to avoid zero
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  if (dt <= 0) dt = 1e-6f;

  // --- SENSOR 1 ---
  int16_t raw_ax1, raw_ay1, raw_az1, raw_gx1, raw_gy1, raw_gz1;
  readRaw(MPU1_ADDR, raw_ax1, raw_ay1, raw_az1, raw_gx1, raw_gy1, raw_gz1);
  float axn1 = (raw_ax1 - ax_off1) / 16384.0f;
  float ayn1 = (raw_ay1 - ay_off1) / 16384.0f;
  float azn1 = (raw_az1 - az_off1) / 16384.0f;
  float gxn1 = (raw_gx1 - gx_off1) / 131.0f;  // deg/s
  float gyn1 = (raw_gy1 - gy_off1) / 131.0f;
  float gzn1 = (raw_gz1 - gz_off1) / 131.0f;
  // Angular acceleration
  float angAccX1 = (gxn1 - prev_gx1) / dt;
  float angAccY1 = (gyn1 - prev_gy1) / dt;
  float angAccZ1 = (gzn1 - prev_gz1) / dt;
  prev_gx1 = gxn1; prev_gy1 = gyn1; prev_gz1 = gzn1;
  // Integrate gyro → quaternion
  float gx_r1 = gxn1*(PI/180.0f), gy_r1 = gyn1*(PI/180.0f), gz_r1 = gzn1*(PI/180.0f);
  float qDot0_1 = -0.5f*(q1_1*gx_r1 + q2_1*gy_r1 + q3_1*gz_r1);
  float qDot1_1 =  0.5f*(q0_1*gx_r1 + q2_1*gz_r1 - q3_1*gy_r1);
  float qDot2_1 =  0.5f*(q0_1*gy_r1 - q1_1*gz_r1 + q3_1*gx_r1);
  float qDot3_1 =  0.5f*(q0_1*gz_r1 + q1_1*gy_r1 - q2_1*gx_r1);
  q0_1 += qDot0_1*dt; q1_1 += qDot1_1*dt; q2_1 += qDot2_1*dt; q3_1 += qDot3_1*dt;
  float norm1 = sqrtf(q0_1*q0_1 + q1_1*q1_1 + q2_1*q2_1 + q3_1*q3_1);
  q0_1/=norm1; q1_1/=norm1; q2_1/=norm1; q3_1/=norm1;
  // Accelerometer correction
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

  // --- SENSOR 2 ---
  int16_t raw_ax2, raw_ay2, raw_az2, raw_gx2, raw_gy2, raw_gz2;
  readRaw(MPU2_ADDR, raw_ax2, raw_ay2, raw_az2, raw_gx2, raw_gy2, raw_gz2);
  float axn2 = (raw_ax2 - ax_off2) / 16384.0f;
  float ayn2 = (raw_ay2 - ay_off2) / 16384.0f;
  float azn2 = (raw_az2 - az_off2) / 16384.0f;
  float gxn2 = (raw_gx2 - gx_off2) / 131.0f;
  float gyn2 = (raw_gy2 - gy_off2) / 131.0f;
  float gzn2 = (raw_gz2 - gz_off2) / 131.0f;
  float angAccX2 = (gxn2 - prev_gx2) / dt;
  float angAccY2 = (gyn2 - prev_gy2) / dt;
  float angAccZ2 = (gzn2 - prev_gz2) / dt;
  prev_gx2 = gxn2; prev_gy2 = gyn2; prev_gz2 = gzn2;
  float gx_r2 = gxn2*(PI/180.0f), gy_r2 = gyn2*(PI/180.0f), gz_r2 = gzn2*(PI/180.0f);
  float qDot0_2 = -0.5f*(q1_2*gx_r2 + q2_2*gy_r2 + q3_2*gz_r2);
  float qDot1_2 =  0.5f*(q0_2*gx_r2 + q2_2*gz_r2 - q3_2*gy_r2);
  float qDot2_2 =  0.5f*(q0_2*gy_r2 - q1_2*gz_r2 + q3_2*gx_r2);
  float qDot3_2 =  0.5f*(q0_2*gz_r2 + q1_2*gy_r2 - q2_2*gx_r2);
  q0_2 += qDot0_2*dt; q1_2 += qDot1_2*dt; q2_2 += qDot2_2*dt; q3_2 += qDot3_2*dt;
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

  // --- Output ---
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

  delay(500);  // 2 Hz updates
}
