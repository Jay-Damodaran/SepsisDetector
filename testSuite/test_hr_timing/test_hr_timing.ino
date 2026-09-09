/*
 * test_hr_timing.ino
 *
 * No hardware required.
 * Tests the non-blocking HR accumulation logic from sepsorMain.
 *
 * The current design does NOT use a blocking calc_HR() function. Instead:
 *   - processBeat() mirrors the beat-detection block inside loop(): accumulates
 *     inter-beat BPMs into bpmAccum/beatCount for the current 2-second window.
 *   - commitWindow() mirrors the readHR commit block: pushes the window average
 *     into hr_running and resets accumulation state.
 *
 * Tests:
 *   T1 - First beat sets lastBeat but contributes no BPM (no delta yet)
 *   T2 - 200 ms delta (300 BPM) rejected by upper guard
 *   T3 - 4000 ms delta (15 BPM) rejected by lower guard
 *   T4 - Valid beat (600 ms) accumulates into bpmAccum and beatCount
 *   T5 - commitWindow with beatCount > 0: hr_running updated, hr_count incremented
 *   T6 - commitWindow with beatCount == 0: hr_running and hr_count unchanged
 *   T7 - 600 ms delta → 100 BPM committed to hr_running
 *   T8 - Multiple windows: hr_running is cumulative sum of per-window averages
 *   T9 - lastBeat reset to 0 after commitWindow; first beat of next window anchors cleanly
 */

// ── mirror of sepsorMain HR state ─────────────────────────────────────────────
unsigned long lastBeat  = 0;
float         bpmAccum  = 0.0f;
uint8_t       beatCount = 0;
float         hr_running = 0.0f;
int           hr_count   = 0;

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

void resetHRState() {
  lastBeat   = 0;
  bpmAccum   = 0.0f;
  beatCount  = 0;
  hr_running = 0.0f;
  hr_count   = 0;
}

// Mirrors the beat-detection block inside loop().
void processBeat(unsigned long beatTime) {
  if (lastBeat != 0) {
    unsigned long delta = beatTime - lastBeat;
    float bpm = 60000.0f / (float)delta;
    if (bpm > 20.0f && bpm < 255.0f) {
      bpmAccum += bpm;
      beatCount++;
    }
  }
  lastBeat = beatTime;
}

// Mirrors the readHR commit block inside loop().
void commitWindow() {
  if (beatCount > 0) {
    hr_running += bpmAccum / beatCount;
    hr_count++;
  }
  bpmAccum  = 0.0f;
  beatCount = 0;
  lastBeat  = 0;
}

// ── test cases ────────────────────────────────────────────────────────────────

void test_first_beat_no_bpm() {
  Serial.println("\n[T1] First beat sets lastBeat, contributes no BPM");
  resetHRState();
  processBeat(1000UL);
  expect(lastBeat  == 1000UL, "lastBeat set to first beat timestamp");
  expect(beatCount == 0,      "beatCount still 0 — no delta on first beat");
  expect(bpmAccum  == 0.0f,   "bpmAccum still 0.0");
}

void test_bpm_above_255_rejected() {
  Serial.println("\n[T2] 200 ms delta (300 BPM) rejected by upper guard");
  resetHRState();
  processBeat(0UL);
  processBeat(200UL); // 60000/200 = 300 BPM → rejected
  expect(beatCount == 0,    "beatCount 0 after 300 BPM beat");
  expect(bpmAccum  == 0.0f, "bpmAccum unchanged after rejected beat");
}

void test_bpm_below_20_rejected() {
  Serial.println("\n[T3] 4000 ms delta (15 BPM) rejected by lower guard");
  resetHRState();
  processBeat(0UL);
  processBeat(4000UL); // 60000/4000 = 15 BPM → rejected
  expect(beatCount == 0,    "beatCount 0 after 15 BPM beat");
  expect(bpmAccum  == 0.0f, "bpmAccum unchanged after rejected beat");
}

void test_valid_beat_accumulated() {
  Serial.println("\n[T4] Valid beat (600 ms) accumulates into bpmAccum");
  resetHRState();
  processBeat(0UL);
  processBeat(600UL); // 60000/600 = 100 BPM → accepted
  expect(beatCount == 1,                     "beatCount incremented to 1");
  expect(fabsf(bpmAccum - 100.0f) < 0.5f,   "bpmAccum ≈ 100 BPM");
}

void test_commit_updates_hr_running() {
  Serial.println("\n[T5] commitWindow with beatCount > 0 updates hr_running");
  resetHRState();
  processBeat(0UL);
  processBeat(600UL); // 100 BPM
  commitWindow();
  expect(hr_count == 1,                      "hr_count incremented to 1");
  expect(fabsf(hr_running - 100.0f) < 0.5f, "hr_running ≈ 100 BPM");
}

void test_commit_empty_window_no_change() {
  Serial.println("\n[T6] commitWindow with beatCount == 0: hr_running unchanged");
  resetHRState();
  hr_running = 80.0f;
  hr_count   = 1;
  commitWindow(); // beatCount == 0 → no update
  expect(hr_count   == 1,     "hr_count unchanged after empty window");
  expect(hr_running == 80.0f, "hr_running unchanged after empty window");
}

void test_600ms_gives_100bpm() {
  Serial.println("\n[T7] Consistent 600 ms beats → 100 BPM committed");
  resetHRState();
  processBeat(0UL);
  processBeat(600UL);
  processBeat(1200UL);
  processBeat(1800UL);
  commitWindow();
  Serial.print("  hr_running: "); Serial.println(hr_running, 2);
  expect(fabsf(hr_running - 100.0f) < 0.5f, "600 ms delta → 100 BPM in hr_running");
}

void test_multiple_windows_cumulative() {
  Serial.println("\n[T8] Multiple windows: hr_running is cumulative sum of window averages");
  resetHRState();
  // Window 1: beats at 600 ms → 100 BPM avg
  processBeat(0UL); processBeat(600UL); processBeat(1200UL);
  commitWindow(); // hr_running = 100, hr_count = 1

  // Window 2: beats at 1000 ms → 60 BPM avg
  processBeat(0UL); processBeat(1000UL); processBeat(2000UL);
  commitWindow(); // hr_running = 160, hr_count = 2

  Serial.print("  hr_running: "); Serial.print(hr_running, 2);
  Serial.print("  hr_count: "); Serial.println(hr_count);
  expect(hr_count == 2,                      "hr_count == 2 after two windows");
  expect(fabsf(hr_running - 160.0f) < 1.0f, "hr_running == 100+60 = 160 after two windows");
}

void test_lastbeat_reset_after_commit() {
  Serial.println("\n[T9] lastBeat reset to 0 after commitWindow");
  resetHRState();
  processBeat(0UL);
  processBeat(600UL);
  commitWindow();
  expect(lastBeat == 0, "lastBeat reset to 0 after window commit");
  // First beat of the new window should anchor without computing a stale delta.
  processBeat(5000UL);
  expect(beatCount == 0, "First beat of new window sets lastBeat, no BPM contributed");
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_hr_timing ===");

  test_first_beat_no_bpm();
  test_bpm_above_255_rejected();
  test_bpm_below_20_rejected();
  test_valid_beat_accumulated();
  test_commit_updates_hr_running();
  test_commit_empty_window_no_change();
  test_600ms_gives_100bpm();
  test_multiple_windows_cumulative();
  test_lastbeat_reset_after_commit();

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");
}

void loop() {}
