const int buttonPin1 = 2;  // First button on pin 2
const int buttonPin2 = 3;  // Second button on pin 3

int buttonState1 = 0;
int buttonState2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
}

void loop() {
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  Serial.print("Button 1: ");
  Serial.print(buttonState1);
  Serial.print(" | Button 2: ");
  Serial.println(buttonState2);

  delay(10);  // Basic debounce and readable serial rate
}
