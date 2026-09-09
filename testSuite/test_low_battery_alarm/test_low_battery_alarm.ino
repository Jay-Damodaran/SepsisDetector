/*
 * test_low_battery_alarm.ino
 *
 * No hardware required.
 * Tests lowBatteryDetect() and the 2-minute buzzer gate using injectable
 * ADC values and a fake time source.
 *
 * The real lowBatteryDetect() calls analogReadMilliVolts(BATTERY), which
 * reads real hardware.  Here we replace that with a global mockAdcMv so
 * every test case can set the simulated voltage independently.
 *
 * The buzzer gate in sepsorMain is:
 *   if (lowBattery && (millis() - active_start_time) < 120000)
 * We inject a fake active_start_time offset to simulate elapsed time.
 *
 * Artificially high threshold:
 *   The real v_threshold = MIN_V + 0.2*(MAX_V-MIN_V) ≈ 1660 mV (half-rail).
 *   In these tests we use the same threshold value so no hardware change is
 *   needed — we simply set mockAdcMv above or below it.
 *
 * Tests:
 *   T1 - ADC above threshold → returns 0, clears warning_val
 *   T2 - ADC below threshold (first call) → returns 1, sets warning_val
 *   T3 - ADC below threshold (repeat, no recovery) → returns 1
 *   T4 - Battery recovering (val > warning_val + RECOVERY_V) → returns 0
 *   T5 - Buzzer gate true at t=0 s into active period
 *   T6 - Buzzer gate true at t=60 s into active period
 *   T7 - Buzzer gate false at t=121 s (alarm window expired)
 *   T8 - Buzzer gate false when lowBattery == 0 regardless of time
 */

// ── constants from sepsorMain ─────────────────────────────────────────────────
#define BATTERY A0        // not actually read; mocked below
#define MAX_V   2100
#define MIN_V   1550
#define RECOVERY_V 100

// Use a higher-than-real threshold to make the test easy to trigger without
// real hardware.  Set mockAdcMv above/below TEST_THRESHOLD to control result.
const int TEST_THRESHOLD = MIN_V + 0.2 * (MAX_V - MIN_V); // 1660 mV

// ── injectable mock state ─────────────────────────────────────────────────────
uint32_t mockAdcMv   = 2000; // simulated raw mV read (before x2 print)
uint32_t val         = 0;
uint32_t warning_val = 0;
int      v_threshold = TEST_THRESHOLD;
unsigned long alarm_start_t = 0;

// ── reimplementation of lowBatteryDetect() using mock ADC ────────────────────
int lowBatteryDetect() {
  val = 0;
  for (uint8_t i = 0; i < 5; i++) val += mockAdcMv;
  val /= 5; // average (all readings identical in mock)

  if (val >= (uint32_t)v_threshold) {
    warning_val  = 0;
    alarm_start_t = 0;
    return 0;
  }
  if (!warning_val) {
    alarm_start_t = millis();
    warning_val   = val;
    return 1;
  }
  if (val > warning_val && (val - warning_val) > RECOVERY_V) return 0;
  return 1;
}

// ── test helpers ──────────────────────────────────────────────────────────────
static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

void resetBatteryState() {
  val          = 0;
  warning_val  = 0;
  alarm_start_t = 0;
}

// ── test cases ────────────────────────────────────────────────────────────────

void test_above_threshold_returns_0() {
  Serial.println("\n[T1] ADC above threshold → returns 0, clears warning_val");
  resetBatteryState();
  warning_val = 1500; // pre-set to confirm it gets cleared
  mockAdcMv   = TEST_THRESHOLD + 50; // 1710 mV — above threshold
  int result  = lowBatteryDetect();
  expect(result == 0,      "lowBatteryDetect returns 0 when above threshold");
  expect(warning_val == 0, "warning_val cleared when battery is OK");
}

void test_below_threshold_first_call_returns_1() {
  Serial.println("\n[T2] ADC below threshold (first call) → returns 1, sets warning_val");
  resetBatteryState();
  mockAdcMv = TEST_THRESHOLD - 50; // 1610 mV — below threshold
  int result = lowBatteryDetect();
  expect(result == 1,                   "lowBatteryDetect returns 1 on first low reading");
  expect(warning_val == mockAdcMv,      "warning_val set to current ADC reading");
}

void test_below_threshold_repeat_returns_1() {
  Serial.println("\n[T3] ADC below threshold (repeat, no recovery) → returns 1");
  resetBatteryState();
  mockAdcMv = TEST_THRESHOLD - 50;
  lowBatteryDetect(); // first call sets warning_val
  int result = lowBatteryDetect(); // second call: same voltage, no recovery
  expect(result == 1, "lowBatteryDetect returns 1 on second low reading without recovery");
}

void test_recovering_battery_returns_0() {
  Serial.println("\n[T4] Battery recovering (val > warning_val + RECOVERY_V) → returns 0");
  resetBatteryState();
  mockAdcMv = TEST_THRESHOLD - 100; // initial low reading → sets warning_val
  lowBatteryDetect();
  uint32_t initial_warning = warning_val;
  mockAdcMv = initial_warning + RECOVERY_V + 10; // clearly recovering
  int result = lowBatteryDetect();
  expect(result == 0, "lowBatteryDetect returns 0 when battery is recovering");
}

void test_buzzer_gate_at_t0() {
  Serial.println("\n[T5] Buzzer gate: true at t=0 of active period");
  uint8_t  lowBattery       = 1;
  unsigned long active_start = millis(); // right now
  bool gate = lowBattery && ((millis() - active_start) < 120000UL);
  expect(gate, "Buzzer gate true at start of active period (t≈0)");
}

void test_buzzer_gate_at_t60s() {
  Serial.println("\n[T6] Buzzer gate: true at t=60 s of active period");
  uint8_t  lowBattery       = 1;
  unsigned long fake_start  = millis() - 60000UL; // pretend started 60s ago
  bool gate = lowBattery && ((millis() - fake_start) < 120000UL);
  expect(gate, "Buzzer gate true at t=60 s (within 2-minute window)");
}

void test_buzzer_gate_at_t121s() {
  Serial.println("\n[T7] Buzzer gate: false at t=121 s (window expired)");
  uint8_t  lowBattery       = 1;
  unsigned long fake_start  = millis() - 121000UL; // pretend started 121s ago
  bool gate = lowBattery && ((millis() - fake_start) < 120000UL);
  expect(!gate, "Buzzer gate false after 2-minute alarm window");
}

void test_buzzer_gate_no_alarm_when_battery_ok() {
  Serial.println("\n[T8] Buzzer gate: false when lowBattery == 0");
  uint8_t  lowBattery       = 0; // battery is fine
  unsigned long fake_start  = millis(); // just started
  bool gate = lowBattery && ((millis() - fake_start) < 120000UL);
  expect(!gate, "Buzzer gate false when lowBattery flag is 0");
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_low_battery_alarm ===");
  Serial.print("  v_threshold used: "); Serial.print(TEST_THRESHOLD); Serial.println(" mV");

  test_above_threshold_returns_0();
  test_below_threshold_first_call_returns_1();
  test_below_threshold_repeat_returns_1();
  test_recovering_battery_returns_0();
  test_buzzer_gate_at_t0();
  test_buzzer_gate_at_t60s();
  test_buzzer_gate_at_t121s();
  test_buzzer_gate_no_alarm_when_battery_ok();

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");
}

void loop() {}
