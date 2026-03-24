#define LOWBATT 1
#define SEPSISWARN 0

const uint8_t buzzPin = 3; 
uint8_t mode;

void setup() {
  pinMode(buzzPin, OUTPUT);
  mode = 1;
}


void playSepsisWarning() {
  tone(buzzPin, 1000);
  delay(1000);
  noTone(buzzPin);
  delay(500);
}

void playLowBattery() {
  tone(buzzPin, 800);
  delay(100);
  noTone(buzzPin);
  delay(100);
  tone(buzzPin, 800);
  delay(100);
  noTone(buzzPin);
  delay(100);
  tone(buzzPin, 800);
  delay(100);
  noTone(buzzPin);
  delay(100);
  delay(1500);
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
