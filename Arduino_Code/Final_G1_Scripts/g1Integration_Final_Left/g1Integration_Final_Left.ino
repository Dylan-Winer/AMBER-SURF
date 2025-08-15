// ===== FSR @ 1000 Hz + MPU6050 (Mahony quaternion + gyro rates from RAW) CSV logger =====
#include <Wire.h>

// ===================== FSRs =====================
const int   analogPins[4]   = {A0, A1, A2, A3};
const float Vcc             = 3.3f;
// Force [N] = slope * Vout + intercept
const float slope[4]        = {308.010f, 252.743f, 232.058f, 236.940f};
const float intercept[4]    = {-413.037f, -339.549f, -313.218f, -319.241f};

// ===================== IMU (MPU-6050) — RAW PATH (same as your “before”) =====================
const uint8_t MPU_addr = 0x68;   // AD0 = GND

// From your IMU script
float A_cal[6] = {265.0f, -80.0f, -700.0f, 0.994f, 1.000f, 1.014f}; // accel offsets/scales
float G_off[3] = { -499.5f, -17.7f, -82.0f};                         // raw gyro offsets (LSB)
#define gscale ((250.f/32768.0f)*(PI/180.0f))                        // LSB -> rad/s

// Mahony state/params
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // qw, qx, qy, qz
float Kp = 30.0f;
float Ki = 0.0f;

// Euler (degrees)
float yaw_deg = 0.0f, pitch_deg = 0.0f, roll_deg = 0.0f;

// ===================== Loop timing =====================
// Fixed 1000 Hz loop (1 ms tick)
const uint32_t LOOP_US = 1000;
uint32_t nextTick = 0;        // micros() timestamp of next tick
unsigned long last_us = 0;    // for Mahony dt

// ===================== Logging Control =====================
bool logging = false;
bool headerPrinted = false;
unsigned long t0_ms = 0;
const uint16_t PRINT_EVERY_N = 1;   // 1 = print every sample (~1000 Hz). Increase to decimate.
uint32_t sampleCount = 0;

// ===================== Helpers =====================
void maybeHandleSerialCommands() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 's' || c == 'S') {
      logging = true;
      headerPrinted = false;   // will print header at next print
      t0_ms = millis();        // reset time zero on start
      sampleCount = 0;
    } else if (c == 'x' || c == 'X') {
      logging = false;
    }
  }
}

void readFSR_Forces(float forces_out[4]) {
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(analogPins[i]);
    float Vout = raw * (Vcc / 1023.0f);
    float Force = slope[i] * Vout + intercept[i];
    if (Force < 0.0f) Force = 0.0f;      // clip negatives
    if (Force < 2.0f) Force = 0.0f;      // deadzone below 2 N
    forces_out[i] = Force;
  }
}

// Read raw accel/gyro from MPU-6050
bool readIMUraw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_addr, (uint8_t)14, (uint8_t)true) != 14) return false;

  int t = Wire.read() << 8; ax = t | Wire.read();
  t = Wire.read() << 8;     ay = t | Wire.read();
  t = Wire.read() << 8;     az = t | Wire.read();
  t = Wire.read() << 8;     (void)(t | Wire.read()); // temp (unused)
  t = Wire.read() << 8;     gx = t | Wire.read();
  t = Wire.read() << 8;     gy = t | Wire.read();
  t = Wire.read() << 8;     gz = t | Wire.read();
  return true;
}

// ---- Mahony update (unchanged math from your IMU code) ----
static void Mahony_update(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float deltat)
{
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // error terms
  float qa, qb, qc;
  static float ix = 0.0f, iy = 0.0f, iz = 0.0f;  // integral feedback
  float tmp;

  tmp = ax*ax + ay*ay + az*az;
  if (tmp > 0.0f) {
    recipNorm = 1.0f / sqrtf(tmp);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // Estimated gravity direction
    vx = q[1]*q[3] - q[0]*q[2];
    vy = q[0]*q[1] + q[2]*q[3];
    vz = q[0]*q[0] - 0.5f + q[3]*q[3];

    // Error between measured and estimated gravity
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix; gy += iy; gz += iz;
    }

    gx += Kp * ex; gy += Kp * ey; gz += Kp * ez;
  }

  // Integrate quaternion rate: q_dot = 0.5 * q ⊗ omega
  const float half_dt = 0.5f * deltat;
  gx *= half_dt; gy *= half_dt; gz *= half_dt;

  qa = q[0]; qb = q[1]; qc = q[2];
  q[0] += (-qb*gx - qc*gy - q[3]*gz);
  q[1] += ( qa*gx + qc*gz - q[3]*gy);
  q[2] += ( qa*gy - qb*gz + q[3]*gx);
  q[3] += ( qa*gz + qb*gy - qc*gx);

  // Normalize quaternion
  recipNorm = 1.0f / sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

void setup() {
  // High baud to sustain ~kHz CSV
  Serial.begin(2000000);
  while (!Serial) { /* wait for USB */ }

  // ADC resolution for FSR scaling (0..1023)
  analogReadResolution(10);

  // I2C setup
  Wire.setSDA(18); Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(5);

  // Wake MPU via register (as in your IMU script)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // wake
  Wire.endTransmission(true);

  nextTick = micros() + LOOP_US;
  last_us  = micros();

  Serial.println("Ready. Type 's' + Enter to START logging CSV; type 'x' + Enter to STOP.");
  Serial.println("(FSR+IMU run at 1000 Hz. Use PRINT_EVERY_N to decimate prints if needed.)");
}

void loop() {
  // Fixed-rate 1000 Hz tick
  const uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) {
    return; // not yet time for the next 1 ms tick
  }
  nextTick += LOOP_US;
  if ((int32_t)(micros() - nextTick) > 5000) {
    nextTick = micros() + LOOP_US;
  }

  maybeHandleSerialCommands();

  // ---------- FSR read ----------
  float forces[4];
  readFSR_Forces(forces);

  // ---------- IMU raw read ----------
  int16_t ax_i=0, ay_i=0, az_i=0, gx_i=0, gy_i=0, gz_i=0;
  bool ok = readIMUraw(ax_i, ay_i, az_i, gx_i, gy_i, gz_i);

  // ---------- Mahony + angular velocity + Euler (all from same raw path) ----------
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  float gx_rad = 0, gy_rad = 0, gz_rad = 0;

  if (ok) {
    // accel with your offsets/scales
    float Axyz[3] = { (float)ax_i, (float)ay_i, (float)az_i };
    for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    // gyro rad/s with your raw offsets + scale
    gx_rad = ((float)gx_i - G_off[0]) * gscale;
    gy_rad = ((float)gy_i - G_off[1]) * gscale;
    gz_rad = ((float)gz_i - G_off[2]) * gscale;

    // dt from micros, like before
    unsigned long now_us = micros();
    float deltat = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    if (deltat <= 0.0f) deltat = 0.001f;

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], gx_rad, gy_rad, gz_rad, deltat);

    // refresh quaternion
    qw = q[0]; qx = q[1]; qy = q[2]; qz = q[3];

    // Euler (same formula/sign/compass wrap as your previous code)
    roll_deg  = atan2f((q[0]*q[1] + q[2]*q[3]),
                       0.5f - (q[1]*q[1] + q[2]*q[2])) * 180.0f / PI;
    pitch_deg = asinf(2.0f * (q[0]*q[2] - q[1]*q[3])) * 180.0f / PI;
    float yaw_tmp = -atan2f((q[1]*q[2] + q[0]*q[3]),
                            0.5f - (q[2]*q[2] + q[3]*q[3])) * 180.0f / PI;
    if (yaw_tmp < 0.0f) yaw_tmp += 360.0f;
    yaw_deg = yaw_tmp;
  }

  // ---------- CSV output ----------
  if (logging) {
    if (!headerPrinted) {
      //Serial.println("time_s,FSR1_N,FSR2_N,FSR3_N,FSR4_N,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg,gx_rad_s,gy_rad_s,gz_rad_s");
      Serial.println("time_s,FSR1_N,FSR2_N,FSR3_N,FSR4_N,qw,qx,qy,qz,gx_rad_s,gy_rad_s,gz_rad_s");
      headerPrinted = true;
    }

    sampleCount++;
    if (sampleCount % PRINT_EVERY_N == 0) {
      const float t_s = (millis() - t0_ms) / 1000.0f;

      Serial.print(t_s, 3); Serial.print(',');
      Serial.print((double)forces[0], 2); Serial.print(',');
      Serial.print((double)forces[1], 2); Serial.print(',');
      Serial.print((double)forces[2], 2); Serial.print(',');
      Serial.print((double)forces[3], 2); Serial.print(',');

      // quaternion
      Serial.print((double)qw, 6); Serial.print(',');
      Serial.print((double)qx, 6); Serial.print(',');
      Serial.print((double)qy, 6); Serial.print(',');
      Serial.print((double)qz, 6); Serial.print(',');

      /*
      // Euler (deg) — order: yaw, pitch, roll
      Serial.print((double)yaw_deg, 3);  Serial.print(',');
      Serial.print((double)pitch_deg, 3);Serial.print(',');
      Serial.print((double)roll_deg, 3); Serial.print(',');
      */

      // gyro (rad/s) from same raw path
      Serial.print((double)gx_rad, 6); Serial.print(',');
      Serial.print((double)gy_rad, 6); Serial.print(',');
      Serial.println((double)gz_rad, 6);
    }
  }
}

