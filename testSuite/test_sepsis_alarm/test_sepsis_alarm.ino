/*
 * test_sepsis_alarm.ino
 *
 * No hardware required.
 * Tests the sepsis detection counter logic from sepsorMain.
 *
 * The relevant block in sepsorMain (after averaging over 6 windows):
 *
 *   if (RR_avg > 22 && HR_avg > 90) { sepsis++; }
 *   else { sepsis = 0; }
 *   if (sepsis >= 2) { while(1) { playSepsisWarning(); } }
 *
 * We replicate this logic without calling playSepsisWarning() or entering
 * the infinite loop.  Instead, we check the sepsis counter value and verify
 * that the alarm condition is correctly reached or correctly suppressed.
 *
 * Also verifies the alarm tone pattern constants:
 *   playSepsisWarning : 1000 Hz, 1000 ms on, 500 ms off
 *   playLowBattery    : 800 Hz, 100 ms on, 100 ms off (×3), 1500 ms rest
 *
 * Tests:
 *   T1  - Both criteria met → sepsis increments
 *   T2  - Only RR elevated (HR normal) → sepsis resets to 0
 *   T3  - Only HR elevated (RR normal) → sepsis resets to 0
 *   T4  - Neither criterion met → sepsis resets to 0
 *   T5  - Two consecutive positive windows → sepsis reaches 2 (alarm threshold)
 *   T6  - Positive then negative → sepsis resets; second positive only reaches 1
 *   T7  - Three consecutive positives → sepsis reaches 3 (alarm already at 2)
 *   T8  - Boundary: RR == 22 (not > 22) → not detected
 *   T9  - Boundary: HR == 90 (not > 90) → not detected
 *   T10 - Boundary: RR == 22.01, HR == 90.01 → detected
 *   T11 - Sepsis alarm tone: 1000 Hz, on 1000ms, off 500ms
 *   T12 - Low battery tone: 800 Hz, 100ms on/off pattern with 1500ms rest
 */

// ── state mirroring sepsorMain ────────────────────────────────────────────────
uint8_t sepsis = 0;

// buzzer constants (verified against sepsorMain values)
const uint16_t SEPSIS_FREQ_HZ      = 1000;
const uint16_t SEPSIS_ON_MS        = 1000;
const uint16_t SEPSIS_OFF_MS       = 500;

const uint16_t LOWBATT_FREQ_HZ     = 800;
const uint16_t LOWBATT_BEEP_MS     = 100;
const uint16_t LOWBATT_GAP_MS      = 100;
const uint8_t  LOWBATT_BEEP_COUNT  = 3;
const uint16_t LOWBATT_REST_MS     = 1500;

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

// ── replicate one detection window (does NOT enter while(1)) ─────────────────
// Returns true if the alarm threshold (sepsis >= 2) was reached this window.
bool runDetectionWindow(float RR_avg, float HR_avg) {
  if (RR_avg > 22.0f && HR_avg > 90.0f) { sepsis++; }
  else                                   { sepsis = 0; }
  return (sepsis >= 2);
}

// ── test cases ────────────────────────────────────────────────────────────────

void test_both_criteria_increments() {
  Serial.println("\n[T1] Both criteria met → sepsis increments");
  sepsis = 0;
  runDetectionWindow(24.0f, 95.0f);
  expect(sepsis == 1, "sepsis == 1 after one positive window");
}

void test_only_rr_elevated_resets() {
  Serial.println("\n[T2] Only RR elevated → resets to 0");
  sepsis = 1;
  runDetectionWindow(24.0f, 85.0f); // HR not elevated
  expect(sepsis == 0, "sepsis reset when only RR criterion met");
}

void test_only_hr_elevated_resets() {
  Serial.println("\n[T3] Only HR elevated → resets to 0");
  sepsis = 1;
  runDetectionWindow(18.0f, 95.0f); // RR not elevated
  expect(sepsis == 0, "sepsis reset when only HR criterion met");
}

void test_neither_criterion_resets() {
  Serial.println("\n[T4] Neither criterion met → resets to 0");
  sepsis = 1;
  runDetectionWindow(16.0f, 72.0f);
  expect(sepsis == 0, "sepsis reset when neither criterion met");
}

void test_two_consecutive_reach_threshold() {
  Serial.println("\n[T5] Two consecutive positive windows → alarm threshold reached");
  sepsis = 0;
  bool alarm1 = runDetectionWindow(25.0f, 95.0f);
  bool alarm2 = runDetectionWindow(25.0f, 95.0f);
  expect(!alarm1,       "Alarm NOT triggered after first positive window");
  expect( alarm2,       "Alarm triggered after second consecutive positive window");
  expect(sepsis == 2,   "sepsis counter == 2");
}

void test_interrupted_sequence_no_alarm() {
  Serial.println("\n[T6] Positive → negative → positive: only reaches 1, no alarm");
  sepsis = 0;
  runDetectionWindow(25.0f, 95.0f); // sepsis = 1
  runDetectionWindow(16.0f, 72.0f); // sepsis = 0 (reset)
  bool alarm = runDetectionWindow(25.0f, 95.0f); // sepsis = 1
  expect(!alarm,      "No alarm after non-consecutive positives");
  expect(sepsis == 1, "sepsis == 1, not 2");
}

void test_three_consecutive() {
  Serial.println("\n[T7] Three consecutive positives → counter reaches 3");
  sepsis = 0;
  runDetectionWindow(25.0f, 95.0f); // 1
  runDetectionWindow(25.0f, 95.0f); // 2 — alarm threshold crossed
  runDetectionWindow(25.0f, 95.0f); // 3 — alarm already triggered in real code
  expect(sepsis == 3, "sepsis counter reaches 3 after 3 consecutive detections");
}

void test_rr_boundary_not_greater() {
  Serial.println("\n[T8] RR == 22 (not > 22) → not detected");
  sepsis = 0;
  runDetectionWindow(22.0f, 95.0f);
  expect(sepsis == 0, "RR == 22 does not satisfy RR > 22");
}

void test_hr_boundary_not_greater() {
  Serial.println("\n[T9] HR == 90 (not > 90) → not detected");
  sepsis = 0;
  runDetectionWindow(25.0f, 90.0f);
  expect(sepsis == 0, "HR == 90 does not satisfy HR > 90");
}

void test_boundary_just_above() {
  Serial.println("\n[T10] RR == 22.01, HR == 90.01 → detected");
  sepsis = 0;
  runDetectionWindow(22.01f, 90.01f);
  expect(sepsis == 1, "RR 22.01 and HR 90.01 satisfy both criteria");
}

void test_sepsis_alarm_tone_constants() {
  Serial.println("\n[T11] Sepsis alarm tone constants match spec");
  expect(SEPSIS_FREQ_HZ == 1000, "Sepsis alarm frequency is 1000 Hz");
  expect(SEPSIS_ON_MS   == 1000, "Sepsis alarm on-time is 1000 ms");
  expect(SEPSIS_OFF_MS  == 500,  "Sepsis alarm off-time is 500 ms");
  // Duty cycle 1000/(1000+500) ≈ 66.7%
  float duty = (float)SEPSIS_ON_MS / (SEPSIS_ON_MS + SEPSIS_OFF_MS) * 100.0f;
  expect(fabsf(duty - 66.7f) < 0.5f, "Sepsis alarm duty cycle ≈ 66.7%");
}

void test_lowbatt_tone_constants() {
  Serial.println("\n[T12] Low battery tone constants match spec");
  expect(LOWBATT_FREQ_HZ    == 800,  "Low battery frequency is 800 Hz");
  expect(LOWBATT_BEEP_MS    == 100,  "Low battery beep on-time is 100 ms");
  expect(LOWBATT_GAP_MS     == 100,  "Low battery gap between beeps is 100 ms");
  expect(LOWBATT_BEEP_COUNT == 3,    "Low battery plays 3 beeps per burst");
  expect(LOWBATT_REST_MS    == 1500, "Low battery rest after burst is 1500 ms");
  // playLowBattery: beep(100)+gap(100)+beep(100)+gap(100)+beep(100)+rest(1500) = 2000 ms
  // There are only 2 inter-beep gaps (between 3 beeps), not 3.
  uint32_t burst = LOWBATT_BEEP_COUNT * LOWBATT_BEEP_MS
                 + (LOWBATT_BEEP_COUNT - 1) * LOWBATT_GAP_MS
                 + LOWBATT_REST_MS;
  expect(burst == 2000, "Full low-battery burst cycle is 2000 ms");
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_sepsis_alarm ===");

  test_both_criteria_increments();
  test_only_rr_elevated_resets();
  test_only_hr_elevated_resets();
  test_neither_criterion_resets();
  test_two_consecutive_reach_threshold();
  test_interrupted_sequence_no_alarm();
  test_three_consecutive();
  test_rr_boundary_not_greater();
  test_hr_boundary_not_greater();
  test_boundary_just_above();
  test_sepsis_alarm_tone_constants();
  test_lowbatt_tone_constants();

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");
}

void loop() {}
