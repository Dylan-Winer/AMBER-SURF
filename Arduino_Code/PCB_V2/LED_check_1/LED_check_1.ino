// Pins to turn on
const int ledPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 28, 29, 33};

void setup() {
  // Set all pins as OUTPUT
  for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], HIGH);  // Turn LED on
  }
}

void loop() {
  // Do nothing
}
