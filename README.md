# AMBER-SURF
Repository hosting project files as part of my Summer Undergraduate Research Fellowship (SURF) 2025 project within the AMBER Lab at Caltech.

## Mechanical Components & Assembly

This section covers the physical build of the foot modules and begins exactly where the right-foot assembly was left off.

---

### Complete the Right Foot: Rubber Sole & Adhesive Nuts

1. **Get the sole DXF**
   - Download `sole_base.DXF` from this repo:  
     https://github.com/Dylan-Winer/AMBER-SURF/blob/main/SolidWorks-CAD/G1_Integration/DXF_G1_3/sole_base.DXF

2. **Cut the rubber sole**
   - Acquire **1/8" thick rubber** sheet.
   - **Waterjet** cut the sole using `sole_base.DXF`. Deburr/trim if needed for a clean perimeter.

3. **Bolt the remaining hardware**
   - From the bag labeled **“4/40 BOLTS & SPACERS”**, locate the **four remaining adhesive nuts**, **4-40 bolts**, and **spacers**.
   - Install these in the positions on the **right foot** where bolts are currently missing. Snug the 4-40 bolts; don’t over-torque.
   - There are through-holes cut into the cable channels on both the heel connectors. Use these to fit an allen key through. You may need to push the cables out of the way with the allen key.

4. **Bond the adhesive nuts to the rubber**
   - Remove the rubber sole; place a small, even bead of **Gorilla Glue** *or* a suitable **epoxy** on the **underside of all 8 adhesive nuts**.
   - Press the **waterjet rubber sole** onto the adhesive nuts with **even pressure**. Using a **vise** to hold the foot makes this easier; protect surfaces with scrap pads.
   - **Cure per adhesive instructions.** (If using Gorilla Glue, lightly dampen the rubber for proper foaming; for epoxy, degrease both surfaces first.)

5. **Prep for base plate + toe installation**
   - **Unscrew** the four **4-40 bolts & spacers** that mount the **heel connector** to the foot so the **right foot matches the left**.
   - The **heel must come off** so the **base plate and toe** can be **slid on from the front**.

> ✅ Quick checklist  
> - [ ] Rubber sole waterjet and is clean  
> - [ ] All 8 adhesive nuts bonded and cured  
> - [ ] Heel connector hardware removed (right foot)  
> - [ ] Fit check: base plate + toe can slide on from the front

---

### Install Each Module (Left & Right Feet)

1. **Slide-on fit**
   - **Slip the G1 foot** through the **toe cover** until the bottom of the G1 foot is **fully seated** on the **top of the bottom base plate**.
   - Verify the **toe and heel bumpers** **fully enclose** the foot with uniform contact.

2. **Reinstall heel hardware**
   - Cover the heel of the G1 with the **heel mounts**.
   - **Screw the four 4-40 bolts + spacers** into the **four heel-mount holes**. Tighten evenly in a cross pattern until snug.

3. **Cable & strain relief**
   - Route cables cleanly along edges and through any provided channels. Add **strain relief** (zip ties/adhesive anchors) to protect connectors and solder joints.

4. **USB to host**
   - Connect **micro-USB** from **each foot module** to the computer. In practice you’ll likely use **USB extension cables** plus **90° micro-USB cables** to keep a low profile.

5. **Proceed to logging**
   - Follow the **Arduino/CoolTerm** instructions in the previous section to set **per-foot baud rates** and **capture data**.

# AMBER Lab SURF 2025 — G1 Foot Sensing Module

High-rate (1000 Hz) force + IMU logging for the Unitree G1 humanoid foot.  
The following contains Teensy 4.1 usage notes to capture synchronized FSR forces and MPU-6050 orientation/gyro data to CSV via CoolTerm.

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
