// analog pin connected to battery + terminal
#define BATTERY A0
// lipo range is 3.1V to 4.2V, but voltage is halved due to voltage divider to ensure safe input to analog pin
#define MAX_V 2100
#define MIN_V 1550
#define RECOVERY_V 100 // stop alarm when battery has recovered by 0.1 V to avoid annoying user


uint32_t val; // current adc val from pin connected to battery + terminal
uint32_t warning_val = 0; // adc val corresponding to initial low battery reading
//int adc_threshold = static_cast<int>((ADC_MAX / ADC_MAX_V) * (MIN_V + 0.2 * (MAX_V - MIN_V))); // ADC threshold corresponding to 20% battery
int v_threshold = MIN_V + 0.2 * (MAX_V - MIN_V);
unsigned long alarm_start_t = 0; // intial time that buzzer goes off

// function that returns 1 if battery is low and not being charged and 0 otherwise
int lowBatteryDetect(){
  val = 0;
  for(uint8_t i = 0; i < 5; i++){
    val += analogReadMilliVolts(BATTERY); // read battery voltage
  }
  val /= 5;
  Serial.println(val * 2);
  delay(200);
  if(val >= v_threshold){ // compare to threshold corresponding to 20% battery
    warning_val = 0;
    alarm_start_t = 0;
    return 0;
  }
  if(!warning_val){
    alarm_start_t = millis(); // note alarm start time if this is the first time low battery is detected
    warning_val = val; // note adc val of warning
    return 1;
  }
  if(val > warning_val && (val - warning_val) > RECOVERY_V){ // return 0 if battery is being charged
    return 0;
  }
  return 1;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
}

int readBattery(){
  val = 0;
  for(uint8_t i = 0; i < 5; i++){
    val += analogReadMilliVolts(BATTERY); // read battery voltage
  }
  val /= 5;
  Serial.println(val * 2);
  delay(200);
  return 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  // low battery detected and alarm has been sounding for less than 2 minutes within a 10 minute period
  // if(lowBatteryDetect() && ((millis() - alarm_start_t) % 600000) < 120000){ 
  //   Serial.println("Sounding buzzer");
  //   //playLowBattery();
  // }
  readBattery();
}
