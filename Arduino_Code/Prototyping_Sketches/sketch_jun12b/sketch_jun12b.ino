const int fsr1Pin = A0;
const int fsr2Pin = A7;
const float R_fixed = 22000.0;
const float Vcc = 3.3;

unsigned long startTime = 0;
bool logging = false;

float computeForce(int raw) {
  float V_fsr = raw * (Vcc / 1023.0);
  if (V_fsr == 0) return 0;
  float R_fsr = (Vcc - V_fsr) / V_fsr * R_fixed;
  float C = 2.58e4;
  float n = 1.16;
  float force = C * pow(R_fsr, -n);
  if (force < 0.2) force = 0;
  if (force > 20.0) force = 20.0;
  return force;
}

void setup() {
  Serial.begin(9600);
  Serial.println("timestamp_ms,force1_N,force2_N");
}

void loop() {
  int raw1 = analogRead(fsr1Pin);
  int raw2 = analogRead(fsr2Pin);
  float force1 = computeForce(raw1);
  float force2 = computeForce(raw2);

  unsigned long now = millis();

  // Start timer if force is detected and not already logging
  if ((force1 > 0 || force2 > 0) && !logging) {
    logging = true;
    startTime = now;
  }

  // If within 5 seconds of activation, record data
  if (logging && (now - startTime <= 5000)) {
    Serial.print(now - startTime);
    Serial.print(",");
    Serial.print(force1, 3);
    Serial.print(",");
    Serial.println(force2, 3);
  }

  // Stop logging after 5 seconds
  if (logging && (now - startTime > 5000)) {
    logging = false;
    Serial.println("Done logging\n");
  }

  delay(20);  // 50 Hz logging rate
}
