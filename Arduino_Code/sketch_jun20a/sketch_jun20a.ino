// LED On at A1 – Teensy 4.0

void setup() {
  // Configure A1 as an output
  pinMode(A6, OUTPUT);
  
  // Drive A1 HIGH to turn the LED on
  digitalWrite(A6, HIGH);
}

void loop() {
  // Nothing to do in the loop — LED stays on
}
