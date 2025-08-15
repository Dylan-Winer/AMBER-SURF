// Combined Adafruit_MPU6050 (gyro rates) + Mahony (quaternion) demo
// Prints only: angular velocities (rad/s) and quaternion
// Keeps Mahony/Euler calculations running in the background

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ------------------ IMU (Adafruit driver) ------------------
Adafruit_MPU6050 mpu;

// ------------------ Mahony filter params/state ------------------
// Quaternion (w, x, y, z)
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
// Gains
static float Kp = 30.0f;
static float Ki = 0.0f;  // leave 0 unless you add integral anti-drift

// For Euler (computed but not printed)
static float yaw_deg = 0, pitch_deg = 0, roll_deg = 0;

// ------------------ Timing ------------------
static unsigned long last_us = 0;      // for deltat
static unsigned long last_ms_print = 0;
const unsigned long print_ms = 200;    // change if you want a different print rate

// ------------------ Mahony AHRS (acc in any linear units, gyro in rad/s) ------------------
static void Mahony_update(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float deltat)
{
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // error terms
  float qa, qb, qc;
  static float ix = 0.0f, iy = 0.0f, iz = 0.0f;  // integral feedback

  // Use accelerometer only if valid (avoids NaN in normalization)
  const float anorm = ax*ax + ay*ay + az*az;
  if (anorm > 0.0f) {
    recipNorm = 1.0f / sqrtf(anorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated gravity direction (half factors absorbed)
    vx = q[1]*q[3] - q[0]*q[2];
    vy = q[0]*q[1] + q[2]*q[3];
    vz = q[0]*q[0] - 0.5f + q[3]*q[3];

    // Error is cross product between estimated and measured gravity
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;
      gy += iy;
      gz += iz;
    }

    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
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
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

// Optional: compute Euler (kept for background computation; not printed)
static void computeEulerFromQ()
{
  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  // pitch (y-axis rotation)
  float sinp = 2.0f * (q[0]*q[2] - q[1]*q[3]);
  if (fabsf(sinp) >= 1.0f)
    pitch_deg = copysignf(90.0f, sinp);
  else
    pitch_deg = asinf(sinp) * 180.0f / PI;

  // yaw (z-axis rotation), note sign conventions vary by frame
  float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) { /* wait for USB if needed */ }

  Serial.println(F("Initializing MPU6050 (Adafruit driver) ..."));
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip"));
    while (1) { delay(10); }
  }
  Serial.println(F("MPU6050 Found!"));

  // Configure ranges/bandwidth similar to your angular velocity demo
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  last_us = micros();

  Serial.println(F("Ready. Printing gyro (rad/s) and quaternion (w,x,y,z)."));
  Serial.println(F("gx_rad_s, gy_rad_s, gz_rad_s, qw, qx, qy, qz"));
}

void loop() {
  // --- Timing ---
  const unsigned long now_us = micros();
  float deltat = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (deltat <= 0.0f) deltat = 1e-3f; // guard

  // --- Read IMU via Adafruit driver ---
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  // accel: m/s^2 ; gyro: rad/s

  // --- Run Mahony update in the background ---
  // You can pass accel in m/s^2 directly (Mahony normalizes it internally)
  Mahony_update(accel.acceleration.x,
                accel.acceleration.y,
                accel.acceleration.z,
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                deltat);

  // (Optional) compute Euler but don't print
  computeEulerFromQ();

  // --- Print only angular velocity + quaternion at a controlled rate ---
  const unsigned long now_ms = millis();
  if (now_ms - last_ms_print >= print_ms) {
    last_ms_print = now_ms;

    // CSV style (easy to log/plot)
    Serial.print(gyro.gyro.x, 6); Serial.print(", ");
    Serial.print(gyro.gyro.y, 6); Serial.print(", ");
    Serial.print(gyro.gyro.z, 6); Serial.print(", ");
    Serial.print(q[0], 6);       Serial.print(", ");
    Serial.print(q[1], 6);       Serial.print(", ");
    Serial.print(q[2], 6);       Serial.print(", ");
    Serial.println(q[3], 6);
  }

  // No blocking delay; loop rate governed by sensor read + print interval
}
