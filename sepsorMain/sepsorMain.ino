/*
 * respiratoryRate.ino
 * 
 * Samples the LSM6DSO32 z-axis at 120 Hz on the Xiao ESP32-C3 and stores
 * readings into a double buffer.
 */

#include <Adafruit_LSM6DSO32.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include "MAX30105.h" 
#include "heartRate.h" 
#include "WiFi.h"
#include "esp_bt.h"

// Sampling and Buffer configurations
const uint32_t SAMPLE_RATE_HZ = 120;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ; // 8333 µs
const uint32_t WINDOW_SEC = 30;
const uint32_t BUFFER_SIZE = SAMPLE_RATE_HZ * WINDOW_SEC; // 3600 samples


// double buffer for acceleration
float accbuf[2][BUFFER_SIZE];

// indeces for double buffer
volatile uint8_t active_buf = 0;
volatile int buf_index = 0; 
volatile bool buf_ready[2] = {false, false};
volatile uint8_t readHR = 0; // semaphore to let main thread know when to read HR

// IMU and time initialization
Adafruit_LSM6DSO32 imu;
unsigned long curr_us = 0;
uint8_t rr_count = 0;

// HR sensor
MAX30105 particleSensor;
const uint32_t hr_total_samples = SAMPLE_RATE_HZ * 2; // 240
int hr_count = 0;

// variables for biometrics
float rr_running = 0.0;
float RR_avg;
float hr_running = 0.0;
float HR_avg;

// pin for buzzer to play sepsis or low battery warning
const uint8_t buzzPin = 5; // corresponds to GPIO number on Xiao ESP32-C3, not the digital pin number

uint8_t lowBattery = 0; // flag for lowBattery
uint8_t sepsis = 0; // sepsis detected counter
uint8_t batteryChecked = 0; // flag for whether battery has been checked during this active period
unsigned long active_start_time = 0; // intial time that low battery buzzer goes off

hw_timer_t *timer0 = NULL;

// ISR to sample RR, switch to other buffer if current buffer is filled, and set semaphore to readHR
void IRAM_ATTR sampleAcc() {
  // read accelerometer sample when timer hits 0, and store z component in the buffer only if the buffer is ready
  if (!buf_ready[active_buf]) {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    accbuf[active_buf][buf_index] = accel.acceleration.z;
    buf_index++;
  }
  // check if active buffer is full
  if (buf_index >= (int)BUFFER_SIZE) {
    buf_ready[active_buf] = true;
    active_buf = 1 - active_buf; // switch buffer
    buf_index  = 0;
  }
  // set flag to get HR reading every 2 seconds
  if(!((buf_index + 1) % hr_total_samples)) {
    readHR = 1;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // disable WiFi
  WiFi.mode(WIFI_OFF);
  // Disable Bluetooth
  btStop();
  esp_bt_controller_disable();

  // ensure that IMU works with I2C
  if (!imu.begin_I2C(0x6A)) {
    Serial.println("ERROR: LSM6DSO32 not found at 0x6A — check wiring.");
    while (1);
  }

  // ensure that HR module works with I2C
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    //UsedefaultI2Cport, 400kHz speed 
    Serial.println("MAX30102 was not found. Pleasecheckwiring/power. ");
    while (1) ; //Infinite loop to stop the program
  }

  // accelerometer configuration
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  // hr sensor configuration
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x24);

  // configure buzzer pin as digital output
  pinMode(buzzPin, OUTPUT); 

  // 1 MHz base frequency (1 µs resolution)
  timer0 = timerBegin(1000000);

  // Attach ISR function
  timerAttachInterrupt(timer0, &sampleAcc);

  // Alarm every 8333 µs ≈ 120.0048 Hz; autoreload, indefinite repetition
  timerAlarm(timer0, 8333, true, 0);
}

void loop() {
  // process the buffer that has been fully written to
  if (buf_ready[1 - active_buf]) {
    buf_ready[1 - active_buf] = false;
    rr_running += calc_RR(1 - active_buf);
    rr_count++;
  }
  // calculate HR every 2s
  if (readHR) {
    hr_running += calc_HR();
    hr_count++;
    readHR = 0;
  }
  // check for low battery once at the start of each active period
  if(!batteryChecked) {
    lowBattery = lowBatteryDetect(); // checks if battery is below 20% and isn't recovering
    batteryChecked = 1; // set flag
    active_start_time = millis(); // record start of active cycle
  }
  // play lowBattery sound for 2 minutes every 15 minutes to not annoy user
  if(lowBattery && (millis() - active_start_time) < 120000) { 
    playLowBattery();
  }
  // after ~3 minutes
  if (rr_count >= 6) {
    RR_avg = rr_running / rr_count; // average RR over the 3 minutes
    HR_avg = hr_running / hr_count; // average HR over the 3 minutes
    if(RR_avg > 22 && HR_avg > 90) { // sepsis risk based on RR and HR
      sepsis++;
    }
    else { // reset if no consecutive detections
      sepsis = 0;
    }
    if(sepsis >= 2) { // play sepsis warning indefinitely if sepsis detected
      while(1) {
        playSepsisWarning();
      }
    }
    // sleep for 12 minutes (720000000 us) if there is no sepsis alarm
    esp_sleep_enable_timer_wakeup(720000000);
    esp_light_sleep_start();
    rr_running = 0.0;
    hr_running = 0.0;
    rr_count = 0;
    hr_count = 0;
    batteryChecked = 0;
  }
}


// FFT configuration
const int DS_FACTOR = 8;            // downsample 120 Hz → 15 Hz
const int N_DS = BUFFER_SIZE / DS_FACTOR; // 450 downsampled points
const int FFT_SIZE = 512;          // next power-of-two ≥ N_DS (zero-pad)
const float DS_RATE_HZ = (float)SAMPLE_RATE_HZ / DS_FACTOR; // 15.0 Hz
const float FREQ_RES = DS_RATE_HZ / FFT_SIZE; // Hz per bin ≈ 0.02930 Hz

const float RR_FREQ_LOW = 0.05;     // lower bound frequency in Hz (~3 breaths/min)
const float RR_FREQ_HIGH = 0.78;     // upper bound frequency in Hz (~47 breaths/min)

// FFT working arrays — static so they live in heap, not the ISR stack
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

/*
 * calc_RR — compute respiratory rate from a completed 30-second z-axis buffer.
 *
 * Steps:
 *   1. Downsample from 120 Hz to 15 Hz (keep every DS_FACTOR-th sample)
 *   2. Apply Hann window to the N_DS downsampled points
 *   3. Zero-pad to FFT_SIZE = 512
 *   4. Run FFT, convert to magnitude spectrum
 *   5. Find the peak bin in [RR_FREQ_LOW, RR_FREQ_HIGH] Hz
 *   6. Convert peak frequency → breaths per minute
 */
float calc_RR(int buf_id) {
  // Downsample + Hann window
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = 0.0;
    vImag[i] = 0.0;
  }

  for (int i = 0; i < N_DS; i++) {
    float sample = accbuf[buf_id][i * DS_FACTOR];
    float hann = 0.5 * (1.0 - cosf(TWO_PI * i / (N_DS - 1)));
    vReal[i] = (double)(sample * hann);
  }

  // Compute FFT magnitudes
  ArduinoFFT<double> fft(vReal, vImag, FFT_SIZE, DS_RATE_HZ);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude(); // vReal[i] now holds magnitude at bin i

  // Find peak in respiratory frequency band
  int bin_low = (int)ceilf(RR_FREQ_LOW  / FREQ_RES); // bin 2
  int bin_high = (int)floorf(RR_FREQ_HIGH / FREQ_RES); // bin 26

  double peak_mag = 0.0;
  int peak_bin = bin_low;
  for (int i = bin_low; i <= bin_high; i++) {
    if (vReal[i] > peak_mag) {
      peak_mag = vReal[i];
      peak_bin = i;
    }
  }

  // Convert from Hz to breaths per minute
  float rr_hz= peak_bin * FREQ_RES;
  float rr_bpm = rr_hz * 60.0;

  Serial.print("Respiratory Rate: ");
  Serial.print(rr_bpm, 1);
  Serial.println(" breaths/min");
  return rr_bpm;
}

const uint8_t RATE_SIZE = 4; // Increase this for more averaging. 4 is good
long lastBeat; // Time at which the last beat occurred
float beatsPerMinute; // Current BPM value
float beatAvg; // variable to hold average hr in bpm
uint8_t hr_collected; // holds current number of samples

// computes average HR over RATE_SIZE samples, discarding the first beat that is based on last beat being at 0ms
uint8_t calc_HR() {
  beatAvg = 0;
  lastBeat = 0;  // force first beat to be discarded (stale delta)
  hr_collected = 0;

  while (hr_collected < RATE_SIZE) {
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue)) {
      if (lastBeat == 0){ // discard first beat
        lastBeat = millis();
        continue;
      }
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        beatAvg += beatsPerMinute;
        hr_collected++;
      }
    }
  }
  beatAvg /= RATE_SIZE;
  return beatAvg;
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


// analog pin connected to battery + terminal
#define BATTERY A0
// lipo range is 3.1V to 4.2V, but voltage is halved due to voltage divider to ensure safe input to analog pin
#define MAX_V 2100
#define MIN_V 1550
#define RECOVERY_V 100 // stop alarm when battery has recovered by 0.1 V to avoid annoying user


uint32_t val; // current adc val from pin connected to battery + terminal
uint32_t warning_val = 0; // adc val corresponding to initial low battery reading
int v_threshold = MIN_V + 0.2 * (MAX_V - MIN_V);
unsigned long alarm_start_t = 0;

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
