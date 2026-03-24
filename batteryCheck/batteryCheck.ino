// analog pin connected to battery + terminal
#define BATTERY A0
// lipo range is 3.1V to 4.2V
#define MAX_V 4.2
#define MIN_V 3.1
#define RECOVERY_V 0.1 // stop alarm when battery has recovered by 0.1 V to avoid annoying user
// ADC characteristics
#define ADC_MAX 1023
#define ADC_MAX_V 5.0


int val;
int warning_val = 0; // adc val corresponding to initial low battery reading
int adc_threshold = static_cast<int>((ADC_MAX / ADC_MAX_V) * (MIN_V + 0.2 * (MAX_V - MIN_V))); // ADC threshold corresponding to 20% battery
unsigned long alarm_start_t = 0; // intial time that buzzer goes off

// function that returns 1 if battery is low and not being charged and 0 otherwise
int lowBatteryDetect(){
  val = analogRead(BATTERY); // read battery voltage
  if(val >= adc_threshold){ // compare to threshold corresponding to 20% battery
    warning_val = 0;
    alarm_start_t = 0;
    return 0;
  }
  if(!warning_val){
    alarm_start_t = millis(); // note alarm start time if this is the first time low battery is detected
    warning_val = val; // note adc val of warning
    return 1;
  }
  if((val - warning_val) > RECOVERY_V * (ADC_MAX / ADC_MAX_V)){ // return 0 if battery is being charged
    return 0;
  }
  return 1;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
  // low battery detected and alarm has been sounding for less than 2 minutes within a 10 minute period
  if(lowBatteryDetect() && ((millis() - alarm_start_t) % 600000) < 120000){ 
    Serial.println("Sounding buzzer");
    //playLowBattery();
  }
  delay(500);
}
