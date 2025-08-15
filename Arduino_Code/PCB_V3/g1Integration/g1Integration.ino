// Teensy/Arduino 1000 Hz logger for 4x FSR + MPU-6050 + Madgwick
#include <Wire.h>
#include <MadgwickAHRS.h>

// ===================== FSRs =====================
const int   analogPins[4]   = {A0, A1, A2, A3};
const char* analogNames[4]  = {"FSR1", "FSR2", "FSR3", "FSR4"};
const float Vcc             = 3.3f;
// Force [N] = slope * Vout + intercept
const float slope[4]        = {308.010f, 252.743f, 232.058f, 236.940f};
const float intercept[4]    = {-413.037f, -339.549f, -313.218f, -319.241f};

// ===================== IMU (MPU-6050) =====================
const uint8_t MPU_ADDR = 0x68;   // AD0 = GND
Madgwick filter;

// Gyro bias (deg/s)
float gx_off = 0, gy_off = 0, gz_off = 0;
// For angular accel derivative (deg/s to deg/s^2)
float prev_gx = 0, prev_gy = 0, prev_gz = 0;
float angAccX = 0, angAccY = 0, angAccZ = 0;  // EMA-smoothed (deg/s^2)
const float kalmanK = 0.2f;                   // EMA factor for ang. accel smoothing

// ===================== Loop timing =====================
// Fixed 1000 Hz loop (1 ms tick)
const uint32_t LOOP_US = 1000;
uint32_t nextTick = 0;   // micros() timestamp of next tick

// ===================== Logging Control =====================
bool logging = false;
bool headerPrinted = false;
unsigned long t0_ms = 0;
const uint16_t PRINT_EVERY_N = 1;   // 1 = print every sample (~1000 Hz). Bump to 10 for ~100 Hz.
uint32_t sampleCount = 0;

// ---------- IMU helpers ----------
void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cReadByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void wakeMPU(uint8_t addr) {
  // PWR_MGMT_1: clear sleep
  i2cWriteByte(addr, 0x6B, 0x00);
  delay(100);
}

void configureMPU_1kHz(uint8_t addr) {
  // CONFIG (0x1A): DLPF_CFG = 0 (260 Hz gyro BW, 1 kHz sample rate)
  i2cWriteByte(addr, 0x1A, 0x00);
  // SMPLRT_DIV (0x19): with DLPF on, base is 1 kHz; divider=0 -> 1 kHz
  i2cWriteByte(addr, 0x19, 0x00);
  // GYRO_CONFIG (0x1B): FS_SEL=0 (±250 dps => 131 LSB/(deg/s))
  i2cWriteByte(addr, 0x1B, 0x00);
  // ACCEL_CONFIG (0x1C): AFS_SEL=0 (±2 g => 16384 LSB/g)
  i2cWriteByte(addr, 0x1C, 0x00);
  delay(50);
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
    Wire.read(); Wire.read(); // TEMP (skip)
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

// GYRO-ONLY calibration: estimate gyro biases (deg/s). Do NOT change accelerometer.
void calibrateGyro(uint8_t addr, float &gx_off, float &gy_off, float &gz_off) {
  const int samples = 2000;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(addr, ax, ay, az, gx, gy, gz);
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delayMicroseconds(500); // quicker calibration but still averages motion-free bias
  }
  gx_off = (sum_gx / (float)samples) / 131.0f;
  gy_off = (sum_gy / (float)samples) / 131.0f;
  gz_off = (sum_gz / (float)samples) / 131.0f;
}

// ===================== Arduino =====================
void setup() {
  // High baud to sustain ~kHz CSV
  Serial.begin(2000000);
  while (!Serial) { /* wait for USB */ }

  // ADC resolution matches scaling (0..1023)
  analogReadResolution(10);

  // I2C for IMU at 400 kHz (MPU-6050 limit)
  Wire.setSDA(18); Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(400000);

  wakeMPU(MPU_ADDR);
  configureMPU_1kHz(MPU_ADDR);
  calibrateGyro(MPU_ADDR, gx_off, gy_off, gz_off);

  // Madgwick nominal sample rate = 1000 Hz
  filter.begin(1000.0f);

  nextTick = micros() + LOOP_US;

  Serial.println("Ready. Type 's' + Enter to START logging CSV; type 'x' + Enter to STOP.");
  Serial.println("(Sensors run at 1000 Hz. Use PRINT_EVERY_N to decimate prints if needed.)");
}

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

void loop() {
  // Fixed-rate 1000 Hz tick
  const uint32_t now = micros();
  // Wait until it's time (keeps phase over serial/other work)
  if ((int32_t)(now - nextTick) < 0) {
    return; // not yet time for the next 1 ms tick
  }
  // Catch up if we fell behind (avoid drift)
  nextTick += LOOP_US;
  // If very far behind (USB burst), realign to now+1ms
  if ((int32_t)(micros() - nextTick) > 5000) {
    nextTick = micros() + LOOP_US;
  }

  maybeHandleSerialCommands();

  // ---------- FSR read (forces only) ----------
  float forces[4];
  readFSR_Forces(forces);

  // ---------- IMU update (assume dt = 0.001 s at 1 kHz) ----------
  {
    int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
    readRaw(MPU_ADDR, ax_i, ay_i, az_i, gx_i, gy_i, gz_i);

    // Accel in g (±2g => 16384 LSB/g). KEEP GRAVITY.
    const float ax = ax_i / 16384.0f;
    const float ay = ay_i / 16384.0f;
    const float az = az_i / 16384.0f;

    // Gyro in deg/s minus bias
    const float gxd = (gx_i / 131.0f) - gx_off;
    const float gyd = (gy_i / 131.0f) - gy_off;
    const float gzd = (gz_i / 131.0f) - gz_off;

    // Angular acceleration via EMA on derivative of gyro (deg/s^2)
    const float dt = 0.001f; // fixed 1 kHz
    const float rawAccX = (gxd - prev_gx) / dt;
    const float rawAccY = (gyd - prev_gy) / dt;
    const float rawAccZ = (gzd - prev_gz) / dt;
    angAccX = (1 - kalmanK) * angAccX + kalmanK * rawAccX;
    angAccY = (1 - kalmanK) * angAccY + kalmanK * rawAccY;
    angAccZ = (1 - kalmanK) * angAccZ + kalmanK * rawAccZ;
    prev_gx = gxd; prev_gy = gyd; prev_gz = gzd;

    // Madgwick fusion (gyro in rad/s, accel in g)
    filter.updateIMU(gxd * DEG_TO_RAD, gyd * DEG_TO_RAD, gzd * DEG_TO_RAD, ax, ay, az);
  }

  // ---------- CSV output (only when logging) ----------
  if (logging) {
    if (!headerPrinted) {
      Serial.println("time_s,FSR1_N,FSR2_N,FSR3_N,FSR4_N,qw,qx,qy,qz,angAccX,angAccY,angAccZ");
      headerPrinted = true;
    }

    // Decimate prints to keep bandwidth manageable if desired
    sampleCount++;
    if (sampleCount % PRINT_EVERY_N == 0) {
      float qw, qx, qy, qz;
      filter.getQuaternion(&qw, &qx, &qy, &qz);

      const float t_s = (millis() - t0_ms) / 1000.0f;

      // CSV row: time, 4 forces, 4 quat, 3 ang acc
      Serial.print(t_s, 3); Serial.print(',');
      Serial.print((double)forces[0], 2); Serial.print(',');
      Serial.print((double)forces[1], 2); Serial.print(',');
      Serial.print((double)forces[2], 2); Serial.print(',');
      Serial.print((double)forces[3], 2); Serial.print(',');
      Serial.print((double)qw, 6);       Serial.print(',');
      Serial.print((double)qx, 6);       Serial.print(',');
      Serial.print((double)qy, 6);       Serial.print(',');
      Serial.print((double)qz, 6);       Serial.print(',');
      Serial.print((double)angAccX, 2);  Serial.print(',');
      Serial.print((double)angAccY, 2);  Serial.println((double)angAccZ, 2);
    }
  }
}
