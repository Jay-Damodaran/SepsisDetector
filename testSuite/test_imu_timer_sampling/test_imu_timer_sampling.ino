/*
 * test_imu_timer_sampling.ino
 *
 * Hardware test — requires LSM6DSO32 wired to Xiao ESP32-C3 via I2C (0x6A).
 *
 * Verifies that the hardware timer fires the ISR at ~120 Hz and that each
 * firing produces a valid, non-stuck IMU z-axis reading.
 *
 * Pass criteria:
 *   - Measured sample rate is within ±2 Hz of 120 Hz
 *   - No two consecutive samples are identical (sensor is live, not frozen)
 *   - All samples fall within ±40 m/s² (plausible accelerometer range)
 *   - No sample is exactly 0.0 for the entire run (not a dead read)
 */

#include <Adafruit_LSM6DSO32.h>
#include <Wire.h>

// ── constants matching sepsorMain ─────────────────────────────────────────────
const uint32_t TARGET_HZ      = 120;
const uint32_t TEST_DURATION_MS = 5000; // measure over 5 seconds

// ── capture buffer (5 s × 120 Hz = 600 samples, well within SRAM) ─────────────
const uint32_t CAP_SIZE = TARGET_HZ * (TEST_DURATION_MS / 1000) + 20;
volatile float  capBuf[CAP_SIZE];
volatile uint32_t capCount  = 0;
volatile uint32_t firstUs   = 0;
volatile uint32_t lastUs    = 0;
volatile bool   capDone     = false;

Adafruit_LSM6DSO32 imu;
hw_timer_t *timer0 = NULL;

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR sampleISR() {
  if (capDone) return;
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  uint32_t now = (uint32_t)micros();
  if (capCount == 0) firstUs = now;
  if (capCount < CAP_SIZE) {
    capBuf[capCount] = accel.acceleration.z;
    lastUs = now;
    capCount++;
  }
  if (capCount >= CAP_SIZE) capDone = true;
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_imu_timer_sampling ===");

  if (!imu.begin_I2C(0x6A)) {
    Serial.println("FATAL: LSM6DSO32 not found at 0x6A — check wiring.");
    while (1);
  }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  // identical timer config to sepsorMain
  timer0 = timerBegin(1000000);
  timerAttachInterrupt(timer0, &sampleISR);
  timerAlarm(timer0, 8333, true, 0);

  Serial.println("Sampling for 5 seconds...");
}

// ── loop: wait for capture then evaluate ──────────────────────────────────────
void loop() {
  if (!capDone) return;

  timerDetachInterrupt(timer0);
  timerEnd(timer0);

  uint32_t n = capCount;
  uint32_t elapsedUs = lastUs - firstUs;
  float elapsedS = elapsedUs / 1e6f;
  float measuredHz = (n > 1) ? (float)(n - 1) / elapsedS : 0.0f;

  Serial.print("  Samples captured : "); Serial.println(n);
  Serial.print("  Elapsed (s)      : "); Serial.println(elapsedS, 4);
  Serial.print("  Measured rate    : "); Serial.print(measuredHz, 2); Serial.println(" Hz");

  // ── T1: sample rate within ±2 Hz of 120 ──────────────────────────────────
  expect(measuredHz >= 118.0f && measuredHz <= 122.0f,
         "Measured sample rate within 118–122 Hz");

  // ── T2: no two consecutive samples are identical ─────────────────────────
  bool allSame = false;
  for (uint32_t i = 1; i < n; i++) {
    if (capBuf[i] == capBuf[i - 1]) { allSame = true; break; }
  }
  expect(!allSame, "No two consecutive samples are identical (sensor live)");

  // ── T3: all samples within ±40 m/s² ──────────────────────────────────────
  bool inRange = true;
  for (uint32_t i = 0; i < n; i++) {
    if (capBuf[i] < -40.0f || capBuf[i] > 40.0f) { inRange = false; break; }
  }
  expect(inRange, "All samples within ±40 m/s²");

  // ── T4: not all samples zero ──────────────────────────────────────────────
  bool allZero = true;
  for (uint32_t i = 0; i < n; i++) {
    if (capBuf[i] != 0.0f) { allZero = false; break; }
  }
  expect(!allZero, "Not all samples are 0.0 (real reads, not dead data)");

  // ── T5: print first 10 samples for visual sanity ─────────────────────────
  Serial.println("  First 10 z-axis samples (m/s²):");
  for (int i = 0; i < 10 && i < (int)n; i++) {
    Serial.print("    ["); Serial.print(i); Serial.print("] ");
    Serial.println(capBuf[i], 4);
  }

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");

  while (1); // halt
}
