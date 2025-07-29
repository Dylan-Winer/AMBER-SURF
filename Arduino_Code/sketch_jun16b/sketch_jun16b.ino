const int fsr1Pin = A0;
const int fsr2Pin = A7;
const float R_fixed = 22000.0;
const float Vcc = 3.3;

float computeForce(int raw) {
  float V_fsr = raw * (Vcc / 1023.0);
  if (V_fsr < 0.001) return 0;  // safer than == 0
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
  Serial.println("timestamp_ms,force1_N,status1,force2_N,status2");
}

void loop() {
  int raw1 = analogRead(fsr1Pin);
  int raw2 = analogRead(fsr2Pin);
  float force1 = computeForce(raw1);
  float force2 = computeForce(raw2);

  String status1 = (force1 > 1) ? "ON" : "OFF";
  String status2 = (force2 > 1) ? "ON" : "OFF";

  unsigned long now = millis();

  Serial.print(now);
  Serial.print(",");

  Serial.print("Front sensor: ");
  //Serial.print(force1, 3);
  Serial.print(",");
  Serial.print(status1);
  Serial.print(",");

  Serial.print("Back sensor: ");
  //Serial.print(force2, 3);
  Serial.print(",");
  Serial.println(status2);

 // delay(100);  // Delay
}
