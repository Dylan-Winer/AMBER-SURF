//
// MPU-6050 Mahony IMU  (yaw angle is relative to starting orientation)
// last update 7/9/2020 
//

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"

Adafruit_MPU6050 mpu;

// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
int MPU_addr = 0x68;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6050. They are not correct for other examples.
//The IMU code will NOT work well or at all if these are not correct

float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 200; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output

void setup() {

  Wire.begin();                 // ensure I2C up before any access
  Wire.setClock(400000);        // 400 kHz Fast-mode
  Wire.setTimeout(5);           // short timeout to avoid hangs

  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) { /* brief wait */ }
  Serial.println("starting");

  // Wake the MPU via register-level path
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // wake
  Wire.endTransmission(true);

  // Initialize Adafruit driver (retry instead of hang)
  int tries = 0;
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip, retrying...");
    delay(200);
    if (++tries > 20) {
      Serial.println("Proceeding without Adafruit driver initialized.");
      break;
    }
  }
  if (tries <= 20) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println("+-2G"); break;
    case MPU6050_RANGE_4_G:  Serial.println("+-4G"); break;
    case MPU6050_RANGE_8_G:  Serial.println("+-8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println("+- 250 deg/s"); break;
    case MPU6050_RANGE_500_DEG:  Serial.println("+- 500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+- 1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+- 2000 deg/s"); break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ:  Serial.println("94 Hz"); break;
    case MPU6050_BAND_44_HZ:  Serial.println("44 Hz"); break;
    case MPU6050_BAND_21_HZ:  Serial.println("21 Hz"); break;
    case MPU6050_BAND_10_HZ:  Serial.println("10 Hz"); break;
    case MPU6050_BAND_5_HZ:   Serial.println("5 Hz"); break;
    }
  }

  Serial.println("");
  delay(100);
}

// AHRS loop

void loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  static bool adafruit_ok = true;  // assume ok after setup; adjust if needed
  if (adafruit_ok) {
    mpu.getEvent(&a, &g, &temp);
  }

  static int i = 0;
  static float deltat = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers

  //raw data containers
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];

  // ---- Robust raw read with availability guard (no goto) ----
  bool ok = true;

  if (ok) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) ok = false;
  }

  if (ok) {
    if (Wire.requestFrom(MPU_addr, 14, true) != 14) ok = false;
  }

  if (ok) {
    int t = Wire.read() << 8; ax = t | Wire.read();
    t = Wire.read() << 8;     ay = t | Wire.read();
    t = Wire.read() << 8;     az = t | Wire.read();
    t = Wire.read() << 8;     Tmp = t | Wire.read(); // temp
    t = Wire.read() << 8;     gx = t | Wire.read();
    t = Wire.read() << 8;     gy = t | Wire.read();
    t = Wire.read() << 8;     gz = t | Wire.read();

    Axyz[0] = (float) ax;
    Axyz[1] = (float) ay;
    Axyz[2] = (float) az;

    //apply offsets and scale factors from Magneto
    for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float) gx - G_off[0]) * gscale; // rad/s
    Gxyz[1] = ((float) gy - G_off[1]) * gscale;
    Gxyz[2] = ((float) gz - G_off[2]) * gscale;

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // Compute Tait-Bryan angles.
    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // conventional yaw increases clockwise from North
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI; if (yaw < 0) yaw += 360.0;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;
  }

  // ---- Print section (unchanged formatting) ----
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;

    // Print quaternion first
    Serial.print("q: ");
    Serial.print(q[0], 6); Serial.print(", ");
    Serial.print(q[1], 6); Serial.print(", ");
    Serial.print(q[2], 6); Serial.print(", ");
    Serial.print(q[3], 6);
    Serial.print("   ");

    // Then print Euler angles (yaw, pitch, roll)
    Serial.print("ypr: ");
    Serial.print(yaw, 0);   Serial.print(", ");
    Serial.print(pitch, 0); Serial.print(", ");
    Serial.println(roll, 0);
    Serial.print("   ");

    if (adafruit_ok) {
      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print(", Y: ");
      Serial.print(g.gyro.y);
      Serial.print(", Z: ");
      Serial.print(g.gyro.z);
      Serial.println(" rad/s");

      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");
    }
  }
}
//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
