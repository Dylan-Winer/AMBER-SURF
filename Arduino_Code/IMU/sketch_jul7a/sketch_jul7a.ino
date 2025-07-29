// MPU6050 Offset Calibration + Pitch/Roll LEDs + Complementary Quaternion + Angular Acceleration

#include <Wire.h>

const uint8_t MPU_ADDR     = 0x68;
const int     externalLED  = 8;

// Calibration & raw data
int16_t ax, ay, az, gx, gy, gz;
float   ax_offset=0, ay_offset=0, az_offset=0;
float   gx_offset=0, gy_offset=0, gz_offset=0;

// Quaternion state
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
unsigned long lastTime;
const float accWeight = 0.02f;     // accelerometer “trust” in complementary filter

// Previous gyro readings for angular acceleration
float prev_gxn = 0, prev_gyn = 0, prev_gzn = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(externalLED, OUTPUT);

  Wire.begin();
  Serial.begin(115200);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  delay(1000);

  // --- Calibration ---
  const int samples = 2000;
  long sum_ax=0, sum_ay=0, sum_az=0;
  long sum_gx=0, sum_gy=0, sum_gz=0;
  Serial.println("Calibrating... keep MPU flat & still");
  for (int i = 0; i < samples; i++) {
    readRaw();
    sum_ax += ax; sum_ay += ay; sum_az += az;
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delay(3);
  }
  ax_offset = sum_ax / (float)samples;
  ay_offset = sum_ay / (float)samples;
  az_offset = sum_az / (float)samples - 16384.0f;  // remove 1 g
  gx_offset = sum_gx / (float)samples;
  gy_offset = sum_gy / (float)samples;
  gz_offset = sum_gz / (float)samples;

  Serial.print("Calibration offsets: ");
  Serial.print(ax_offset); Serial.print(", ");
  Serial.print(ay_offset); Serial.print(", ");
  Serial.print(az_offset); Serial.print(", ");
  Serial.print(gx_offset); Serial.print(", ");
  Serial.print(gy_offset); Serial.print(", ");
  Serial.println(gz_offset);

  lastTime = micros();
}

void loop() {
  // --- Read & normalize ---
  readRaw();
  float axn = (ax - ax_offset) / 16384.0f;
  float ayn = (ay - ay_offset) / 16384.0f;
  float azn = (az - az_offset) / 16384.0f;
  float gxn = (gx - gx_offset) / 131.0f;  // deg/s
  float gyn = (gy - gy_offset) / 131.0f;
  float gzn = (gz - gz_offset) / 131.0f;

  // --- Pitch & roll for LEDs ---
  float pitch = atan2f(axn, sqrtf(ayn*ayn + azn*azn)) * 180.0f/PI;
  float roll  = atan2f(ayn, azn)              * 180.0f/PI;
  digitalWrite(LED_BUILTIN, fabsf(pitch) > 10.0f ? HIGH : LOW);
  digitalWrite(externalLED, fabsf(roll)  >  6.0f ? HIGH : LOW);

  // --- Timing ---
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;  // seconds
  lastTime = now;

  // --- Angular acceleration (deg/s^2) ---
  float angAccX = (gxn - prev_gxn) / dt;
  float angAccY = (gyn - prev_gyn) / dt;
  float angAccZ = (gzn - prev_gzn) / dt;
  prev_gxn = gxn;
  prev_gyn = gyn;
  prev_gzn = gzn;

  // --- Gyro‐integrate quaternion ---
  float gx_r = gxn * (PI/180.0f);
  float gy_r = gyn * (PI/180.0f);
  float gz_r = gzn * (PI/180.0f);

  float qDot0 = -0.5f*(q1*gx_r + q2*gy_r + q3*gz_r);
  float qDot1 =  0.5f*(q0*gx_r + q2*gz_r - q3*gy_r);
  float qDot2 =  0.5f*(q0*gy_r - q1*gz_r + q3*gx_r);
  float qDot3 =  0.5f*(q0*gz_r + q1*gy_r - q2*gx_r);

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // Normalize
  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  // --- Accelerometer correction ---
  float roll_acc  = atan2f(ayn, azn);
  float pitch_acc = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));
  float cr = cosf(roll_acc * 0.5f), sr = sinf(roll_acc * 0.5f);
  float cp = cosf(pitch_acc * 0.5f), sp = sinf(pitch_acc * 0.5f);

  float q_acc0 = cp*cr;
  float q_acc1 = cp*sr;
  float q_acc2 = sp*cr;
  float q_acc3 = -sp*sr;

  q0 = (1-accWeight)*q0 + accWeight*q_acc0;
  q1 = (1-accWeight)*q1 + accWeight*q_acc1;
  q2 = (1-accWeight)*q2 + accWeight*q_acc2;
  q3 = (1-accWeight)*q3 + accWeight*q_acc3;

  // Normalize again
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  // --- Output ---
  Serial.print("Quaternion: ");
  Serial.print(q0, 6); Serial.print(',');
  Serial.print(q1, 6); Serial.print(',');
  Serial.print(q2, 6); Serial.print(',');
  Serial.println(q3, 6);

  Serial.print("Angular Acceleration (X, Y, Z) (deg/s^2): ");
  Serial.print(angAccY, 3); Serial.print(',');
  Serial.print(angAccX, 3); Serial.print(',');
  Serial.println(angAccZ, 3);

  delay(500);
}

void readRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() >= 14) {
    ax = (Wire.read()<<8)|Wire.read();
    ay = (Wire.read()<<8)|Wire.read();
    az = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read();    // temp skip
    gx = (Wire.read()<<8)|Wire.read();
    gy = (Wire.read()<<8)|Wire.read();
    gz = (Wire.read()<<8)|Wire.read();
  }
}
