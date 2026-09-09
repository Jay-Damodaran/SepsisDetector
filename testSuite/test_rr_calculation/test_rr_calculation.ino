/*
 * test_rr_calculation.ino
 *
 * No hardware required — injects synthetic sinusoids into accbuf and
 * validates that calc_RR() returns the expected respiratory rate.
 *
 * calc_RR() is copied verbatim from sepsorMain, including the bandpower
 * threshold that returns 0 when the device is not worn (power < 1.0).
 *
 * Tests:
 *   T1 - 20 bpm sinusoid  → result within ±2 bpm of 20
 *   T2 - 30 bpm sinusoid  → result within ±2 bpm of 30
 *   T3 - 12 bpm sinusoid  → result within ±2 bpm of 12 (low end of band)
 *   T4 - 45 bpm sinusoid  → result within ±2 bpm of 45 (near high end)
 *   T5 - DC flat buffer   → either bandpower-filtered (0) or peak at band min
 *   T6 - 90 bpm sinusoid  → NOT detected inside respiratory band
 *   T7 - calc_RR reads from the correct buffer id
 *   T8 - Zero-amplitude buffer → bandpower < threshold → returns 0 (not worn)
 */

#include <arduinoFFT.h>

// ── constants matching sepsorMain ─────────────────────────────────────────────
const uint32_t SAMPLE_RATE_HZ = 120;
const uint32_t WINDOW_SEC     = 30;
const uint32_t BUFFER_SIZE    = SAMPLE_RATE_HZ * WINDOW_SEC; // 3600

const int   DS_FACTOR   = 8;
const int   N_DS        = BUFFER_SIZE / DS_FACTOR;            // 450
const int   FFT_SIZE    = 512;
const float DS_RATE_HZ  = (float)SAMPLE_RATE_HZ / DS_FACTOR; // 15.0
const float FREQ_RES    = DS_RATE_HZ / FFT_SIZE;              // ~0.02930 Hz/bin

const float RR_FREQ_LOW  = 0.05f;
const float RR_FREQ_HIGH = 0.78f;

// ── buffers ───────────────────────────────────────────────────────────────────
float  accbuf[2][BUFFER_SIZE];
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

// ── buffer fill helpers ───────────────────────────────────────────────────────
void fillSine(int buf_id, float freq_hz, float amplitude = 1.0f) {
  for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
    float t = (float)i / SAMPLE_RATE_HZ;
    accbuf[buf_id][i] = amplitude * sinf(TWO_PI * freq_hz * t);
  }
}

void fillDC(int buf_id, float value) {
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    accbuf[buf_id][i] = value;
}

// ── exact copy of calc_RR from sepsorMain (includes bandpower check) ──────────
float calc_RR(int buf_id) {
  for (int i = 0; i < FFT_SIZE; i++) { vReal[i] = 0.0; vImag[i] = 0.0; }

  for (int i = 0; i < N_DS; i++) {
    float sample = accbuf[buf_id][i * DS_FACTOR];
    float hann   = 0.5f * (1.0f - cosf(TWO_PI * i / (N_DS - 1)));
    vReal[i]     = (double)(sample * hann);
  }

  ArduinoFFT<double> fft(vReal, vImag, FFT_SIZE, DS_RATE_HZ);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  int bin_low  = (int)ceilf(RR_FREQ_LOW  / FREQ_RES);
  int bin_high = (int)floorf(RR_FREQ_HIGH / FREQ_RES);

  // Bandpower check: integrate power (magnitude²) via trapezoidal rule.
  // Mirrors Python: np.trapz(|rfft(az_down)|², freqs). Returns 0 if not worn.
  double bandpower = 0.0;
  for (int i = bin_low; i < bin_high; i++) {
    double p0 = vReal[i]     * vReal[i];
    double p1 = vReal[i + 1] * vReal[i + 1];
    bandpower += 0.5 * (p0 + p1) * FREQ_RES;
  }
  if (bandpower < 1.0) return 0.0f;

  double peak_mag = 0.0;
  int    peak_bin = bin_low;
  for (int i = bin_low; i <= bin_high; i++) {
    if (vReal[i] > peak_mag) { peak_mag = vReal[i]; peak_bin = i; }
  }

  float rr_hz  = peak_bin * FREQ_RES;
  float rr_bpm = rr_hz * 60.0f;
  return rr_bpm;
}

// ── test cases ────────────────────────────────────────────────────────────────

void test_20bpm() {
  Serial.println("\n[T1] 20 bpm sinusoid → ~20 bpm");
  fillSine(0, 20.0f / 60.0f);
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr >= 18.0f && rr <= 22.0f, "20 bpm within ±2 bpm of 20");
}

void test_30bpm() {
  Serial.println("\n[T2] 30 bpm sinusoid → ~30 bpm");
  fillSine(0, 30.0f / 60.0f);
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr >= 28.0f && rr <= 32.0f, "30 bpm within ±2 bpm of 30");
}

void test_12bpm_low_end() {
  Serial.println("\n[T3] 12 bpm sinusoid → ~12 bpm (low end of band)");
  fillSine(0, 12.0f / 60.0f);
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr >= 10.0f && rr <= 14.0f, "12 bpm within ±2 bpm of 12");
}

void test_45bpm_high_end() {
  Serial.println("\n[T4] 45 bpm sinusoid → ~45 bpm (near high end of band)");
  fillSine(0, 45.0f / 60.0f);
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr >= 43.0f && rr <= 47.0f, "45 bpm within ±2 bpm of 45");
}

void test_dc_buffer_no_spurious_peak() {
  Serial.println("\n[T5] DC flat buffer → no spurious high-bpm peak");
  fillDC(0, 1.0f);
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  // DC + Hann window may fail the bandpower check (rr == 0) or produce a result
  // at the lowest valid bin (rr ≈ min_bpm). Both are acceptable — no high-bpm spike.
  float min_bpm = ((int)ceilf(RR_FREQ_LOW / FREQ_RES)) * FREQ_RES * 60.0f;
  expect(rr == 0.0f || rr <= min_bpm + 4.0f,
         "DC: bandpower-filtered (0) or result at band minimum, no spurious spike");
}

void test_90bpm_out_of_band() {
  Serial.println("\n[T6] 90 bpm sinusoid → not detected inside respiratory band");
  fillSine(0, 90.0f / 60.0f); // 1.5 Hz — above RR_FREQ_HIGH = 0.78 Hz
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr < 50.0f, "90 bpm out-of-band tone does not produce ~90 bpm result");
}

void test_uses_correct_buffer_id() {
  Serial.println("\n[T7] calc_RR reads from the requested buffer id");
  fillSine(0, 20.0f / 60.0f); // buf 0: 20 bpm
  fillSine(1, 30.0f / 60.0f); // buf 1: 30 bpm
  float rr0 = calc_RR(0);
  float rr1 = calc_RR(1);
  Serial.print("  buf0: "); Serial.print(rr0, 2);
  Serial.print(" bpm  buf1: "); Serial.print(rr1, 2); Serial.println(" bpm");
  expect(rr0 >= 18.0f && rr0 <= 22.0f, "buf 0 gives ~20 bpm");
  expect(rr1 >= 28.0f && rr1 <= 32.0f, "buf 1 gives ~30 bpm");
}

void test_zero_signal_not_worn() {
  Serial.println("\n[T8] Zero-amplitude buffer → bandpower < threshold → returns 0");
  fillDC(0, 0.0f); // all zeros
  float rr = calc_RR(0);
  Serial.print("  calc_RR returned: "); Serial.print(rr, 2); Serial.println(" bpm");
  expect(rr == 0.0f, "Zero signal returns 0 (not worn / no respiratory signal)");
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_rr_calculation ===");

  test_20bpm();
  test_30bpm();
  test_12bpm_low_end();
  test_45bpm_high_end();
  test_dc_buffer_no_spurious_peak();
  test_90bpm_out_of_band();
  test_uses_correct_buffer_id();
  test_zero_signal_not_worn();

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");
}

void loop() {}
