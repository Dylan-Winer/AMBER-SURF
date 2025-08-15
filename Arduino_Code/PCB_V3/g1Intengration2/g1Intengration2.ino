// Teensy/Arduino 1000 Hz logger for 4x FSR only
#include <Wire.h>

// ===================== FSRs =====================
const int   analogPins[4]   = {A0, A1, A2, A3};
const char* analogNames[4]  = {"FSR1", "FSR2", "FSR3", "FSR4"};
const float Vcc             = 3.3f;
// Force [N] = slope * Vout + intercept
const float slope[4]        = {308.010f, 252.743f, 232.058f, 236.940f};
const float intercept[4]    = {-413.037f, -339.549f, -313.218f, -319.241f};

// ===================== Loop timing =====================
// Fixed 1000 Hz loop (1 ms tick)
const uint32_t LOOP_US = 1000;
uint32_t nextTick = 0;   // micros() timestamp of next tick

// ===================== Logging Control =====================
bool logging = false;
bool headerPrinted = false;
unsigned long t0_ms = 0;
const uint16_t PRINT_EVERY_N = 1;   // 1 = print every sample (~1000 Hz)
uint32_t sampleCount = 0;

// ===================== Arduino =====================
void setup() {
  // High baud to sustain ~kHz CSV
  Serial.begin(2000000);
  while (!Serial) { /* wait for USB */ }

  // ADC resolution matches scaling (0..1023)
  analogReadResolution(10);

  nextTick = micros() + LOOP_US;

  Serial.println("Ready. Type 's' + Enter to START logging CSV; type 'x' + Enter to STOP.");
  Serial.println("(FSRs run at 1000 Hz. Use PRINT_EVERY_N to decimate prints if needed.)");
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

  // ---------- CSV output (only when logging) ----------
  if (logging) {
    if (!headerPrinted) {
      Serial.println("time_s,FSR1_N,FSR2_N,FSR3_N,FSR4_N");
      headerPrinted = true;
    }

    sampleCount++;
    if (sampleCount % PRINT_EVERY_N == 0) {
      const float t_s = (millis() - t0_ms) / 1000.0f;

      Serial.print(t_s, 3); Serial.print(',');
      Serial.print((double)forces[0], 2); Serial.print(',');
      Serial.print((double)forces[1], 2); Serial.print(',');
      Serial.print((double)forces[2], 2); Serial.print(',');
      Serial.println((double)forces[3], 2);
    }
  }
}
