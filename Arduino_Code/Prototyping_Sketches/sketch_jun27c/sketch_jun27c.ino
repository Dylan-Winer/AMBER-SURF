// MPU6050 Offset Calibration for Teensy 4.0
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const int externalLED = 8;
int16_t ax, ay, az, gx, gy, gz;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);     // onboard LED
  pinMode(externalLED, OUTPUT);     // external LED

  Wire.begin();
  Serial.begin(115200);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // Let the sensor stabilize
  delay(1000);

  const int samples = 2000;
  long sum_ax=0, sum_ay=0, sum_az=0;
  long sum_gx=0, sum_gy=0, sum_gz=0;

  Serial.println("Calibrating... keep MPU flat & still");

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    if (Wire.available() >= 14) {
      ax = (Wire.read() << 8) | Wire.read();
      ay = (Wire.read() << 8) | Wire.read();
      az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); // temp skip
      gx = (Wire.read() << 8) | Wire.read();
      gy = (Wire.read() << 8) | Wire.read();
      gz = (Wire.read() << 8) | Wire.read();
      
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
    }
    delay(3);
  }

  ax_offset = sum_ax / (float)samples;
  ay_offset = sum_ay / (float)samples;
  az_offset = sum_az / (float)samples - 16384.0;  // subtract 1g
  gx_offset = sum_gx / (float)samples;
  gy_offset = sum_gy / (float)samples;
  gz_offset = sum_gz / (float)samples;

  Serial.println("Offsets:");
  Serial.print("ax ay az gx gy gz = ");
  Serial.print(ax_offset); Serial.print(" ");
  Serial.print(ay_offset); Serial.print(" ");
  Serial.print(az_offset); Serial.print(" ");
  Serial.print(gx_offset); Serial.print(" ");
  Serial.print(gy_offset); Serial.print(" ");
  Serial.println(gz_offset);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    // Offset correction, scaling
    float axn = (ax - ax_offset) / 16384.0;
    float ayn = (ay - ay_offset) / 16384.0;
    float azn = (az - az_offset) / 16384.0;
    float gxn = (gx - gx_offset) / 131.0;
    float gyn = (gy - gy_offset) / 131.0;
    float gzn = (gz - gz_offset) / 131.0;

    Serial.print("Acc (g): ");
    Serial.print(axn,3); Serial.print(", ");
    Serial.print(ayn,3); Serial.print(", ");
    Serial.print(azn,3);
    Serial.print("   Gyro (°/s): ");
    Serial.print(gxn,3); Serial.print(", ");
    Serial.print(gyn,3); Serial.print(", ");
    Serial.println(gzn,3);

    float pitch = atan2(axn, sqrt(ayn * ayn + azn * azn)) * 180.0 / PI;
    float roll  = atan2(ayn, azn) * 180.0 / PI;

    Serial.print("   Pitch (°): ");
    Serial.print(pitch, 1);
    Serial.print("   Roll (°): ");
    Serial.println(roll, 1);

    // Threshold checking
    float pitch_threshold = 10.0;
    float roll_threshold  = 6.0;

    // Onboard LED for pitch
    if (abs(pitch) > pitch_threshold) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }

    // External LED for roll
    if (abs(roll) > roll_threshold) {
      digitalWrite(externalLED, HIGH);
    } else {
      digitalWrite(externalLED, LOW);
    }


  }
  delay(200);
}