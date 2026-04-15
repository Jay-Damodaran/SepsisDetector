#define LOWBATT 1
#define SEPSISWARN 0

// pin for buzzer
const uint8_t buzzPin = 3; 
uint8_t mode;

// setup for buzzer
void setup() {
  pinMode(buzzPin, OUTPUT);
  mode = 1;
}

// function to play 1000Hz sepsis warning with 66% duty cycle repeated over 1.5s
void playSepsisWarning() {
  tone(buzzPin, 1000);
  delay(1000);
  noTone(buzzPin);
  delay(500);
}

// function to play 3 consecutive 800Hz beeps to indicate low battery 
void playLowBattery() {
  tone(buzzPin, 800);
  delay(100); // beeps spaced 100ms apart
  noTone(buzzPin);
  delay(100);
  tone(buzzPin, 800);
  delay(100);
  noTone(buzzPin);
  delay(100);
  tone(buzzPin, 800);
  delay(100);
  noTone(buzzPin);
  delay(1500); // 1.5s until next burst
}

void loop() {
  switch(mode){
    case SEPSISWARN:{
      playSepsisWarning();
      break;
    }
    case LOWBATT:{
      playLowBattery();
      break;
    }
  }
}
