/*
 * respiratoryRate.ino
 *
 * Samples the LSM6DSO32 z-axis at 120 Hz on the Xiao ESP32-C3 and stores
 * readings into a double buffer.
 */

#include <Adafruit_LSM6DSO32.h>
#include <arduinoFFT.h>

// Sampling and Buffer configurations
const uint32_t SAMPLE_RATE_HZ = 120;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ; // 8333 µs
const uint32_t WINDOW_SEC = 30;
const uint32_t BUFFER_SIZE = SAMPLE_RATE_HZ * WINDOW_SEC; // 3600 samples

// FFT configuration
const int DS_FACTOR = 8;            // downsample 120 Hz → 15 Hz
const int N_DS = BUFFER_SIZE / DS_FACTOR; // 450 downsampled points
const int FFT_SIZE = 512;          // next power-of-two ≥ N_DS (zero-pad)
const float DS_RATE_HZ = (float)SAMPLE_RATE_HZ / DS_FACTOR; // 15.0 Hz
const float FREQ_RES = DS_RATE_HZ / FFT_SIZE; // Hz per bin ≈ 0.02930 Hz

const float RR_FREQ_LOW = 0.05;     // lower bound frequency in Hz (~3 breaths/min)
const float RR_FREQ_HIGH = 0.78;     // upper bound frequency in Hz (~47 breaths/min)

// FFT working arrays — static so they live in heap, not the ISR stack
static double vReal[FFT_SIZE];
static double vImag[FFT_SIZE];

// double buffer for acceleration
float accbuf[2][BUFFER_SIZE];
// indeces for double buffer
volatile int active_buf = 0;
volatile int buf_index = 0;
volatile bool buf_ready[2] = {false, false};

// IMU and time initialization
Adafruit_LSM6DSO32 imu;
unsigned long curr_us = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!imu.begin_I2C(0x6A)) {
    Serial.println("ERROR: LSM6DSO32 not found at 0x6A — check wiring.");
    while (1);
  }

  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  curr_us = micros();
}

void loop() {
  unsigned long now = micros();
  if ((now - curr_us) >= SAMPLE_PERIOD_US) {
    curr_us = now;

    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    accbuf[active_buf][buf_index] = accel.acceleration.z;
    buf_index++;
  }

  // check if active buffer is full
  if (buf_index >= (int)BUFFER_SIZE) {
    buf_ready[active_buf] = true;
    active_buf = 1 - active_buf;
    buf_index  = 0;
  }

  // process the buffer that has been fully written to
  int idle_buf = 1 - active_buf;
  if (buf_ready[idle_buf]) {
    buf_ready[idle_buf] = false;
    calcRR(idle_buf);
  }
}

/*
 * calcRR — compute respiratory rate from a completed 30-second z-axis buffer.
 *
 * Steps:
 *   1. Downsample from 120 Hz to 15 Hz (keep every DS_FACTOR-th sample)
 *   2. Apply Hann window to the N_DS downsampled points
 *   3. Zero-pad to FFT_SIZE = 512
 *   4. Run FFT, convert to magnitude spectrum
 *   5. Find the peak bin in [RR_FREQ_LOW, RR_FREQ_HIGH] Hz
 *   6. Convert peak frequency → breaths per minute
 */
void calcRR(int buf_id) {
  // ── 1 & 2: Downsample + Hann window → vReal; zero-fill remainder ──────────
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = 0.0;
    vImag[i] = 0.0;
  }

  for (int i = 0; i < N_DS; i++) {
    float sample = accbuf[buf_id][i * DS_FACTOR];
    float hann = 0.5f * (1.0f - cosf(TWO_PI * i / (N_DS - 1)));
    vReal[i] = (double)(sample * hann);
  }

  // ── 3 & 4: FFT → magnitude spectrum ─────────────────────────────────────
  ArduinoFFT<double> fft(vReal, vImag, FFT_SIZE, DS_RATE_HZ);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude(); // vReal[i] now holds magnitude at bin i

  // ── 5: Find peak in respiratory frequency band ────────────────────────────
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

  // ── 6: Convert to breaths per minute ─────────────────────────────────────
  float rr_hz= peak_bin * FREQ_RES;
  float rr_bpm = rr_hz * 60.0f;

  Serial.print("Respiratory Rate: ");
  Serial.print(rr_bpm, 1);
  Serial.println(" breaths/min");
}
