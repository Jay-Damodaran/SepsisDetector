/*
 * diag_buzzer.ino
 *
 * Hardware required: passive buzzer on GPIO 5 (BUZZ_PIN).
 *
 * Exercises both buzzer patterns from sepsorMain so you can verify tone,
 * timing, and the non-blocking state machine. Runs each pattern 3 times
 * then switches. Serial output annotates each phase.
 *
 * Low-battery pattern  (sepsorMain updateLowBattery):
 *   3 × 100 ms beep at 800 Hz, 100 ms gaps, 1500 ms rest = 2000 ms total.
 *
 * Sepsis-warning pattern (sepsorMain playSepsisWarning — blocking in main,
 *   reproduced here non-blocking for demo):
 *   1000 ms on at 1000 Hz, 500 ms off = 1500 ms total cycle.
 */

const uint8_t BUZZ_PIN = 5;

// ── low-battery state machine (verbatim from sepsorMain) ─────────────────────
enum BuzzState { BUZZ_IDLE, BUZZ_ON, BUZZ_GAP, BUZZ_REST };
BuzzState     buzzState      = BUZZ_IDLE;
uint8_t       buzzBeepCount  = 0;
unsigned long buzzPhaseStart = 0;

void updateLowBattery() {
  unsigned long curr = millis();
  switch (buzzState) {
    case BUZZ_IDLE:
      tone(BUZZ_PIN, 800);
      buzzBeepCount  = 1;
      buzzPhaseStart = curr;
      buzzState      = BUZZ_ON;
      break;
    case BUZZ_ON:
      if (curr - buzzPhaseStart >= 100) {
        noTone(BUZZ_PIN);
        buzzPhaseStart = curr;
        buzzState      = BUZZ_GAP;
      }
      break;
    case BUZZ_GAP:
      if (curr - buzzPhaseStart >= 100) {
        if (buzzBeepCount < 3) {
          tone(BUZZ_PIN, 800);
          buzzBeepCount++;
          buzzPhaseStart = curr;
          buzzState      = BUZZ_ON;
        } else {
          buzzPhaseStart = curr;
          buzzState      = BUZZ_REST;
        }
      }
      break;
    case BUZZ_REST:
      if (curr - buzzPhaseStart >= 1500) {
        buzzState = BUZZ_IDLE;
      }
      break;
  }
}

// ── demo sequencer ────────────────────────────────────────────────────────────
uint8_t       demoPhase     = 0; // 0 = low-battery, 1 = sepsis
uint8_t       cyclesDone    = 0;
bool          sepsisOn      = false;
unsigned long sepsisPhStart = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== diag_buzzer ===");
  Serial.println("Phase 0: Low-battery pattern (3 bursts)");
  pinMode(BUZZ_PIN, OUTPUT);
}

void loop() {
  if (demoPhase == 0) {
    bool wasIdle = (buzzState == BUZZ_IDLE);
    updateLowBattery();
    // Count completed bursts: IDLE→not-IDLE is the start of a burst;
    // track completion by catching the return to BUZZ_IDLE.
    static bool inBurst = false;
    if (!wasIdle && buzzState == BUZZ_IDLE) {
      cyclesDone++;
      inBurst = false;
      Serial.print("  Low-battery burst "); Serial.println(cyclesDone);
    } else if (wasIdle && buzzState != BUZZ_IDLE) {
      inBurst = true;
    }

    if (cyclesDone >= 3) {
      noTone(BUZZ_PIN);
      cyclesDone   = 0;
      demoPhase    = 1;
      sepsisOn     = false;
      sepsisPhStart = millis();
      Serial.println("\nPhase 1: Sepsis-warning pattern (3 cycles)");
    }

  } else {
    unsigned long curr = millis();
    if (!sepsisOn) {
      if (curr - sepsisPhStart >= 500) { // off period
        tone(BUZZ_PIN, 1000);
        sepsisOn      = true;
        sepsisPhStart = curr;
      }
    } else {
      if (curr - sepsisPhStart >= 1000) { // on period
        noTone(BUZZ_PIN);
        sepsisOn      = false;
        sepsisPhStart = curr;
        cyclesDone++;
        Serial.print("  Sepsis cycle "); Serial.println(cyclesDone);

        if (cyclesDone >= 3) {
          cyclesDone   = 0;
          demoPhase    = 0;
          buzzState    = BUZZ_IDLE;
          buzzBeepCount = 0;
          Serial.println("\nPhase 0: Low-battery pattern (3 bursts)");
        }
      }
    }
  }
}
