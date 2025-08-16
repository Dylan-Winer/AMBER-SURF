# AMBER-SURF
Repository hosting project files as part of my Summer Undergraduate Research Fellowship (SURF) 2025 project within the AMBER Lab at Caltech.

# AMBER Lab SURF 2025 — G1 Foot Sensing Module

High-rate (1000 Hz) force + IMU logging for the Unitree G1 humanoid foot.  
This repository contains Teensy 4.1 firmware and usage notes to capture synchronized FSR forces and MPU-6050 orientation/gyro data to CSV via CoolTerm.

---

## TL;DR (How to run & log)

1. **Install & open CoolTerm.**
2. **Connect the Teensy (USB).**
3. In CoolTerm: **Connection → Options…**
   - **Port:** select the COM port associated with your Teensy.
   - **Baudrate:** set to the same value used in firmware `Serial.begin(BAUDRATE)` (defaults shown in the code below are `2,000,000`).
   - Click **OK**.
4. Click **Connect**.
5. Start file logging: **Connection → File Capture → Start…**
   - Choose a folder, **create/name a `.txt` file**, click **Save**.
6. In the CoolTerm terminal, press **`s`** to **start** the program (CSV header prints once).
7. **Run your G1 test** (walk/stance as needed).
8. Press **`x`** to **stop** the program.
9. Stop file logging: **Connection → File Capture → Stop**.

> **Left vs Right foot**  
> You will run **two Teensies** (left/right) at **different baud rates** to avoid USB bandwidth contention and to keep ports distinct.  
> - Edit `Serial.begin(BAUDRATE)` in each foot’s firmware (e.g., Left `2,000,000`, Right `1,500,000`).  
> - In CoolTerm, pick the matching COM port and set the **Baudrate** accordingly for each run.

---

## Data format (CSV columns)

When logging is active, the firmware prints:
time_s,FSR1_N,FSR2_N,FSR3_N,FSR4_N,qw,qx,qy,qz,gx_rad_s,gy_rad_s,gz_rad_s

- **time_s** — seconds since you pressed `s`
- **FSR*_N** — force in Newtons after per-sensor linear calibration and dead-zone/zero-clipping (<2 N → 0)
- **qw..qz** — unit quaternion from Mahony filter
- **g*_rad_s** — body-frame gyro rates (rad/s) derived from RAW with offsets & scale

---

## Hardware overview

- **Controller:** Teensy 4.1 (3.3 V logic)
- **FSRs:** 4× FlexiForce A201 (per-sensor linear calibration applied)
- **IMU:** MPU-6050
- **Loop rate:** 1000 Hz fixed tick (1 ms)

---

## Build & Flash (Teensy 4.x)

1. Open this repo’s firmware in **Arduino IDE** or **Arduino CLI** with **Teensyduino** installed.
2. Select **Board: Teensy 4.0/4.1** and the correct **Port**.
3. (Optional) Adjust:
   - `Serial.begin(BAUDRATE)` for your left/right feet.
   - FSR calibration `slope[]` and `intercept[]`.
   - Mahony gains `Kp`, `Ki`.
4. **Upload**.
