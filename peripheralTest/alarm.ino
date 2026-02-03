const int buzzPin = 3; 

void setup() {
  // put your setup code here, to run once:
  pinMode(buzzPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  tone(buzzPin, 1000);
  delay(1000);
  noTone(buzzPin);
  delay(500);
}
