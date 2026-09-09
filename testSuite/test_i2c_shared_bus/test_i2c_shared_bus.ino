/*
 * test_i2c_shared_bus.ino
 *
 * Hardware test — requires both LSM6DSO32 (0x6A) and MAX30105 on I2C.
 *
 * Verifies that concurrent IMU timer sampling and HR reads don't corrupt
 * the I2C bus.  Mirrors the interleaving that happens in sepsorMain:
 *   - Hardware timer fires sampleAcc() at 120 Hz into double buffer
 *   - Every HR_TOTAL_SAMPLES ticks the ISR sets readHR
 *   - Main loop calls calc_HR() when readHR is set
 *
 * Pass criteria (per cycle, 3 cycles total):
 *   - Devices still respond on I2C after each HR read
 *   - IMU z-values in [-40, +40] m/s²
 *   - HR BPM in [20, 255]
 *   - buf_index advances monotonically across the HR read window
 */

#include <Adafruit_LSM6DSO32.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ── constants matching sepsorMain ─────────────────────────────────────────────
const uint32_t SAMPLE_RATE_HZ   = 120;
const uint32_t WINDOW_SEC       = 30;
const uint32_t BUFFER_SIZE      = SAMPLE_RATE_HZ * WINDOW_SEC; // 3600
const uint32_t HR_TOTAL_SAMPLES = SAMPLE_RATE_HZ * 2;          // 240
const uint8_t  RATE_SIZE        = 4;
const uint8_t  TEST_CYCLES      = 3;

// ── double buffer ─────────────────────────────────────────────────────────────
float accbuf[2][BUFFER_SIZE];
volatile uint8_t active_buf   = 0;
volatile int     buf_index    = 0;
volatile bool    buf_ready[2] = {false, false};
volatile uint8_t readHR       = 0;

Adafruit_LSM6DSO32 imu;
MAX30105 particleSensor;
hw_timer_t *timer0 = NULL;

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

// ── ISR — identical to sepsorMain ────────────────────────────────────────────
void IRAM_ATTR sampleAcc() {
  if (!buf_ready[active_buf]) {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    accbuf[active_buf][buf_index] = accel.acceleration.z;
    buf_index++;
  }
  if (buf_index >= (int)BUFFER_SIZE) {
    buf_ready[active_buf] = true;
    active_buf = 1 - active_buf;
    buf_index  = 0;
  }
  if (!((buf_index + 1) % HR_TOTAL_SAMPLES)) {
    readHR = 1;
  }
}

// ── HR calc — identical to sepsorMain ────────────────────────────────────────
float calc_HR() {
  long  lastBeat = 0;
  float beatAvg  = 0;
  uint8_t collected = 0;

  unsigned long deadline = millis() + 15000; // 15-second timeout
  while (collected < RATE_SIZE && millis() < deadline) {
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue)) {
      if (lastBeat == 0) { lastBeat = millis(); continue; }
      long delta = millis() - lastBeat;
      lastBeat = millis();
      float bpm = 60.0f / (delta / 1000.0f);
      if (bpm < 255 && bpm > 20) {
        beatAvg += bpm;
        collected++;
      }
    }
  }
  if (collected == 0) return 0.0f;
  return beatAvg / collected;
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_i2c_shared_bus ===");

  if (!imu.begin_I2C(0x6A)) {
    Serial.println("FATAL: LSM6DSO32 not found at 0x6A");
    while (1);
  }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FATAL: MAX30105 not found — check wiring.");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x24);

  timer0 = timerBegin(1000000);
  timerAttachInterrupt(timer0, &sampleAcc);
  timerAlarm(timer0, 8333, true, 0);

  Serial.println("Timer started. Running 3 HR-read cycles...\n");
}

// ── loop ─────────────────────────────────────────────────────────────────────
static uint8_t cycle = 0;

void loop() {
  if (cycle >= TEST_CYCLES) return;

  // wait for the ISR to raise the readHR flag
  if (!readHR) return;
  readHR = 0;

  int idx_before = buf_index;
  Serial.print("\n[Cycle "); Serial.print(cycle + 1); Serial.println("]");

  // ── T1: both devices still respond on I2C ────────────────────────────────
  bool imuAlive = imu.begin_I2C(0x6A);
  bool hrAlive  = particleSensor.begin(Wire, I2C_SPEED_FAST);
  expect(imuAlive, "IMU still responds on I2C after timer running");
  expect(hrAlive,  "MAX30105 still responds on I2C after timer running");

  // ── T2: read HR ───────────────────────────────────────────────────────────
  float bpm = calc_HR();
  Serial.print("  HR reading: "); Serial.print(bpm, 1); Serial.println(" BPM");
  expect(bpm >= 20.0f && bpm <= 255.0f, "HR BPM in plausible range [20, 255]");

  // ── T3: sample a few IMU values directly and check range ─────────────────
  bool imuInRange = true;
  for (int i = 0; i < 5; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    if (a.acceleration.z < -40.0f || a.acceleration.z > 40.0f) {
      imuInRange = false; break;
    }
  }
  expect(imuInRange, "IMU z-axis reads within ±40 m/s² after HR read");

  // ── T4: buf_index advanced during the HR read window ─────────────────────
  int idx_after = buf_index;
  expect(idx_after != idx_before || buf_ready[0] || buf_ready[1],
         "buf_index advanced (timer not stalled) during HR read");

  cycle++;

  if (cycle >= TEST_CYCLES) {
    timerDetachInterrupt(timer0);
    timerEnd(timer0);
    Serial.println("\n==============================");
    Serial.print("Results: ");
    Serial.print(passed); Serial.print(" passed, ");
    Serial.print(failed); Serial.println(" failed");
  }
}
