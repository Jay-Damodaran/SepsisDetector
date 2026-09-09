/*
 * test_imu_double_buffer.ino
 *
 * Pure logic test — no hardware required.
 * Exercises the double-buffer state machine from sepsorMain.
 *
 * Matches the CURRENT sepsorMain design where:
 *   - The ISR sets readAcc/readHR flags and checks for buffer flip, but does
 *     NOT write samples or increment buf_index.
 *   - The main loop services readAcc: writes the sample and increments buf_index.
 *
 * simulateISR()        mirrors the ISR body exactly.
 * simulateMainLoop()   mirrors the readAcc branch in loop().
 * tick(sample)         calls both in sequence (one full sampling cycle).
 *
 * Tests:
 *   T1 - First sample writes to buffer 0
 *   T2 - ISR does not set readAcc when active buffer is locked
 *   T3 - Buffer flips after BUFFER_SIZE writes (flip triggers on the NEXT ISR
 *        call after buf_index reaches BUFFER_SIZE, not during the write itself)
 *   T4 - Writes after flip go to buffer 1, not buffer 0
 *   T5 - readAcc not set when buf 1 is locked
 *   T6 - readHR raised every HR_TOTAL_SAMPLES ticks
 *   T7 - readHR not raised in the 238 ticks before the first cadence trigger
 *   T8 - buf_index increases by 1 per tick for the first 10 ticks
 */

const uint32_t SAMPLE_RATE_HZ   = 120;
const uint32_t WINDOW_SEC       = 30;
const uint32_t BUFFER_SIZE      = SAMPLE_RATE_HZ * WINDOW_SEC; // 3600
const uint32_t HR_TOTAL_SAMPLES = SAMPLE_RATE_HZ * 2;          // 240

float accbuf[2][BUFFER_SIZE];

volatile uint8_t active_buf   = 0;
volatile int     buf_index    = 0;
volatile bool    buf_ready[2] = {false, false};
volatile uint8_t readHR       = 0;
volatile uint8_t readAcc      = 0;

static int passed = 0;
static int failed = 0;

void expect(bool condition, const char* label) {
  if (condition) { Serial.print("  PASS: "); passed++; }
  else           { Serial.print("  FAIL: "); failed++; }
  Serial.println(label);
}

void resetState() {
  active_buf   = 0;
  buf_index    = 0;
  buf_ready[0] = false;
  buf_ready[1] = false;
  readHR       = 0;
  readAcc      = 0;
  for (int b = 0; b < 2; b++)
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
      accbuf[b][i] = 0.0f;
}

// Mirrors the ISR: sets flags, flips buffer when buf_index >= BUFFER_SIZE.
// Does NOT write samples or increment buf_index.
void simulateISR() {
  if (!((buf_index + 1) % (int)HR_TOTAL_SAMPLES)) readHR = 1;
  if (!buf_ready[active_buf]) readAcc = 1;
  if (buf_index >= (int)BUFFER_SIZE) {
    buf_ready[active_buf] = true;
    active_buf = 1 - active_buf;
    buf_index  = 0;
  }
}

// Mirrors the readAcc branch in loop(): writes sample and increments buf_index.
void simulateMainLoop(float sample) {
  if (readAcc) {
    readAcc = 0;
    accbuf[active_buf][buf_index] = sample;
    buf_index++;
  }
}

// One complete sampling cycle: ISR fires, then main loop services it.
void tick(float sample) {
  simulateISR();
  simulateMainLoop(sample);
}

// ── test cases ────────────────────────────────────────────────────────────────

void test_initial_write_goes_to_buf0() {
  Serial.println("\n[T1] First sample writes to buffer 0");
  resetState();
  tick(1.23f);
  expect(accbuf[0][0] == 1.23f, "sample stored in accbuf[0][0]");
  expect(buf_index    == 1,     "buf_index incremented to 1");
  expect(active_buf   == 0,     "active_buf still 0");
  expect(buf_ready[0] == false, "buf_ready[0] still false");
}

void test_readAcc_not_set_when_locked() {
  Serial.println("\n[T2] ISR does not set readAcc when active buffer is locked");
  resetState();
  buf_ready[0] = true;   // lock buf 0 before ISR fires
  simulateISR();
  expect(readAcc      == 0,    "readAcc not set when active buffer is locked");
  expect(accbuf[0][0] == 0.0f, "no write attempted to locked buffer");
}

void test_buffer_flip_on_full() {
  Serial.println("\n[T3] Buffer flips after BUFFER_SIZE writes");
  resetState();
  // BUFFER_SIZE ticks produces BUFFER_SIZE writes; buf_index reaches BUFFER_SIZE.
  // The flip does not trigger during these ticks — it triggers on the NEXT
  // simulateISR() call that sees buf_index >= BUFFER_SIZE.
  for (uint32_t i = 0; i < BUFFER_SIZE; i++) tick((float)i);
  expect(active_buf   == 0,              "still on buf 0 after BUFFER_SIZE writes");
  expect(buf_index    == (int)BUFFER_SIZE, "buf_index == BUFFER_SIZE before flip tick");
  expect(buf_ready[0] == false,          "buf_ready[0] false before flip tick");

  simulateISR(); // sees buf_index >= BUFFER_SIZE → flip
  expect(buf_ready[0] == true,  "buf_ready[0] set after flip");
  expect(active_buf   == 1,     "active_buf flipped to 1");
  expect(buf_index    == 0,     "buf_index reset to 0 after flip");
}

void test_writes_go_to_new_buffer_after_flip() {
  Serial.println("\n[T4] Writes after flip go to buffer 1");
  resetState();
  for (uint32_t i = 0; i < BUFFER_SIZE; i++) tick(0.0f);
  simulateISR(); // trigger flip
  tick(7.77f);   // first write to buf 1
  expect(accbuf[1][0] == 7.77f, "sample written to accbuf[1][0] after flip");
  expect(accbuf[0][0] == 0.0f,  "accbuf[0][0] unchanged after flip");
}

void test_no_overwrite_when_buf1_locked() {
  Serial.println("\n[T5] readAcc not set when buf 1 is locked");
  resetState();
  for (uint32_t i = 0; i < BUFFER_SIZE; i++) tick(1.0f);
  simulateISR();     // flip to buf 1
  buf_ready[1] = true; // lock buf 1
  simulateISR();     // should not set readAcc
  expect(readAcc      == 0,    "readAcc not set when buf 1 is locked");
  expect(accbuf[1][0] == 0.0f, "locked buf 1 not overwritten");
}

void test_readHR_raised_at_correct_cadence() {
  Serial.println("\n[T6] readHR raised every HR_TOTAL_SAMPLES ticks");
  resetState();
  int triggers = 0;
  uint32_t limit = HR_TOTAL_SAMPLES * 3;
  for (uint32_t i = 0; i < limit; i++) {
    readHR = 0;
    tick(0.0f);
    if (readHR) triggers++;
  }
  expect(triggers == 3, "readHR raised exactly 3 times in 3 HR windows");
}

void test_readHR_not_raised_before_cadence() {
  Serial.println("\n[T7] readHR not raised in 238 ticks before first cadence trigger");
  resetState();
  bool spurious = false;
  // readHR fires when ISR sees buf_index == HR_TOTAL_SAMPLES-1 = 239.
  // That happens on tick 240. Running HR_TOTAL_SAMPLES-2 = 238 ticks is safe.
  for (uint32_t i = 0; i < HR_TOTAL_SAMPLES - 2; i++) {
    readHR = 0;
    tick(0.0f);
    if (readHR) { spurious = true; break; }
  }
  expect(!spurious, "readHR not raised in 238 ticks before first cadence");
}

void test_buf_index_monotonically_increases() {
  Serial.println("\n[T8] buf_index increases by 1 per tick for first 10 ticks");
  resetState();
  bool monotonic = true;
  int prev = buf_index;
  for (int i = 0; i < 10; i++) {
    tick((float)i);
    if (buf_index != prev + 1) { monotonic = false; break; }
    prev = buf_index;
  }
  expect(monotonic, "buf_index increments by 1 for each of the first 10 ticks");
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== test_imu_double_buffer ===");

  test_initial_write_goes_to_buf0();
  test_readAcc_not_set_when_locked();
  test_buffer_flip_on_full();
  test_writes_go_to_new_buffer_after_flip();
  test_no_overwrite_when_buf1_locked();
  test_readHR_raised_at_correct_cadence();
  test_readHR_not_raised_before_cadence();
  test_buf_index_monotonically_increases();

  Serial.println("\n==============================");
  Serial.print("Results: ");
  Serial.print(passed); Serial.print(" passed, ");
  Serial.print(failed); Serial.println(" failed");
}

void loop() {}
