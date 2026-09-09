/*
 * diag_rr_bandpower.ino
 *
 * Hardware required: LSM6DSO32 on I2C.
 *
 * Collects 30 s of IMU z-axis data at 120 Hz, then computes respiratory rate
 * (RR) and band-power using the exact same algorithm as sepsorMain. Prints
 * both to Serial and repeats. Place the sensor on your chest to get a valid
 * RR reading; leave it on a flat surface to verify the "NOT WORN" detection.
 *
 * Expected output after each 30 s window:
 *   RR: 14.6 bpm  |  Bandpower: 312.45  (threshold: 1.0)
 *   -- or --
 *   RR: NOT WORN  |  Bandpower: 0.02  (threshold: 1.0)
 */

#include <Adafruit_LSM6DSO32.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include "WiFi.h"
#include "esp_bt.h"

// ── constants matching sepsorMain ─────────────────────────────────────────────
const uint32_t SAMPLE_RATE_HZ   = 120;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
const uint32_t WINDOW_SEC       = 30;
const uint32_t BUFFER_SIZE      = SAMPLE_RATE_HZ * WINDOW_SEC;

const int   DS_FACTOR  = 8;
const int   N_DS       = BUFFER_SIZE / DS_FACTOR;
const int   FFT_SIZE   = 512;
const float DS_RATE_HZ = (float)SAMPLE_RATE_HZ / DS_FACTOR;
const float FREQ_RES   = DS_RATE_HZ / FFT_SIZE;

const float RR_FREQ_LOW        = 0.05f;
const float RR_FREQ_HIGH       = 0.78f;
const float BANDPOWER_THRESHOLD = 1.0f;

// ── buffers ───────────────────────────────────────────────────────────────────
float  accbuf[BUFFER_SIZE];
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

// ── ISR state ─────────────────────────────────────────────────────────────────
volatile int     buf_index = 0;
volatile bool    buf_full  = false;
volatile uint8_t readAcc   = 0;

Adafruit_LSM6DSO32 imu;
hw_timer_t        *timer0 = NULL;

void IRAM_ATTR sampleISR() {
  if (!buf_full) readAcc = 1;
}

// ── RR + bandpower computation ────────────────────────────────────────────────
// Returns RR in bpm (0 = not worn). Writes bandpower to out parameter.
float calc_RR_diag(float &bandpower_out) {
  for (int i = 0; i < FFT_SIZE; i++) { vReal[i] = 0.0; vImag[i] = 0.0; }

  float mean = 0.0f;
  for (int i = 0; i < N_DS; i++) mean += accbuf[i * DS_FACTOR];
  mean /= N_DS;

  for (int i = 0; i < N_DS; i++) {
    float sample = accbuf[i * DS_FACTOR] - mean;
    float hann   = 0.5f * (1.0f - cosf(TWO_PI * i / (N_DS - 1)));
    vReal[i]     = (double)(sample * hann);
  }

  ArduinoFFT<double> fft(vReal, vImag, FFT_SIZE, DS_RATE_HZ);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  int bin_low  = (int)ceilf(RR_FREQ_LOW  / FREQ_RES);
  int bin_high = (int)floorf(RR_FREQ_HIGH / FREQ_RES);

  double bandpower = 0.0;
  for (int i = bin_low; i < bin_high; i++) {
    double p0 = vReal[i]     * vReal[i];
    double p1 = vReal[i + 1] * vReal[i + 1];
    bandpower += 0.5 * (p0 + p1) * FREQ_RES;
  }
  bandpower_out = (float)bandpower;

  if (bandpower < BANDPOWER_THRESHOLD) return 0.0f;

  double peak_mag = 0.0;
  int    peak_bin = bin_low;
  for (int i = bin_low; i <= bin_high; i++) {
    if (vReal[i] > peak_mag) { peak_mag = vReal[i]; peak_bin = i; }
  }
  return peak_bin * FREQ_RES * 60.0f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== diag_rr_bandpower ===");
  Serial.println("Collecting 30 s windows. Place sensor flat on chest.");
  Serial.println("Each window takes 30 s to fill before printing.\n");

  WiFi.mode(WIFI_OFF);
  btStop();

  if (!imu.begin_I2C(0x6A)) {
    Serial.println("ERROR: IMU not found. Check wiring.");
    while (1);
  }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  timer0 = timerBegin(1000000);
  timerAttachInterrupt(timer0, &sampleISR);
  timerAlarm(timer0, SAMPLE_PERIOD_US, true, 0);
}

void loop() {
  if (readAcc && !buf_full) {
    readAcc = 0;
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    accbuf[buf_index] = accel.acceleration.z;
    buf_index++;
    if (buf_index >= (int)BUFFER_SIZE) buf_full = true;
  }

  if (buf_full) {
    timerStop(timer0);

    float bandpower;
    float rr = calc_RR_diag(bandpower);

    Serial.print("RR: ");
    if (rr == 0.0f) {
      Serial.print("NOT WORN");
    } else {
      Serial.print(rr, 1);
      Serial.print(" bpm");
    }
    Serial.print("  |  Bandpower: ");
    Serial.print(bandpower, 2);
    Serial.print("  (threshold: ");
    Serial.print(BANDPOWER_THRESHOLD, 1);
    Serial.println(")");

    buf_index = 0;
    buf_full  = false;
    readAcc   = 0;

    timerStart(timer0);
  }
}
