#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Two IMUs on addresses 0x68 (AD0=GND) and 0x69 (AD0=3V3), using Wire (pins 18/19)
MPU6050 mpu1(0x68, &Wire);
MPU6050 mpu2(0x69, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // ——— Configure Wire for pins 18 (SDA) / 19 (SCL) ———
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();

  // ——— Power: be sure VCC → 3V3, not VIN ———

  // ——— Initialize both IMUs ———
  mpu1.initialize();
  mpu2.initialize();

  // ——— (Optional) configure ranges ———
  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu1.setFullScaleGyroRange (MPU6050_GYRO_FS_250);
  mpu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu2.setFullScaleGyroRange (MPU6050_GYRO_FS_250);

  // ——— Sanity check ———
  if (!mpu1.testConnection()) {
    Serial.println("ERROR: MPU6050 #1 (0x68) not responding");
  }
  if (!mpu2.testConnection()) {
    Serial.println("ERROR: MPU6050 #2 (0x69) not responding");
  }
  Serial.println("Dual-IMU on Wire (pins 18/19) ready.");
}

void loop() {
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;

  // Read both IMUs
  mpu1.getMotion6(&ax1,&ay1,&az1,&gx1,&gy1,&gz1);
  mpu2.getMotion6(&ax2,&ay2,&az2,&gx2,&gy2,&gz2);

  // Print IMU1
  Serial.print("IMU1 A=");
    Serial.print(ax1); Serial.print(','); Serial.print(ay1); Serial.print(','); Serial.print(az1);
  Serial.print(" G=");
    Serial.print(gx1); Serial.print(','); Serial.print(gy1); Serial.print(','); Serial.println(gz1);

  // Print IMU2
  Serial.print("IMU2 A=");
    Serial.print(ax2); Serial.print(','); Serial.print(ay2); Serial.print(','); Serial.print(az2);
  Serial.print(" G=");
    Serial.print(gx2); Serial.print(','); Serial.print(gy2); Serial.print(','); Serial.println(gz2);

  delay(100);  // ~10 Hz
}
