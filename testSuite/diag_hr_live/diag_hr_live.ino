/*
 * diag_hr_live.ino
 *
 * Hardware required: MAX30105 on I2C.
 *
 * Polls the HR sensor continuously and prints:
 *   - BPM on each detected beat
 *   - 2-second window average (mirrors sepsorMain's accumulation approach)
 *
 * Place a fingertip firmly on the sensor. IR value is also printed so you
 * can confirm contact (IR should be > ~50000 when finger is present).
 */

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

unsigned long lastBeat   = 0;
float         bpmAccum   = 0.0f;
uint8_t       beatCount  = 0;
unsigned long windowStart = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== diag_hr_live ===");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 not found. Check wiring.");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x24);

  windowStart = millis();
  Serial.println("Place finger firmly on sensor. Waiting for beats...\n");
}

void loop() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    // No finger detected; reset state and wait
    if (millis() - windowStart >= 2000) {
      Serial.println("[No finger detected]");
      lastBeat    = 0;
      bpmAccum    = 0.0f;
      beatCount   = 0;
      windowStart = millis();
    }
    return;
  }

  if (checkForBeat(irValue)) {
    unsigned long now = millis();
    if (lastBeat != 0) {
      unsigned long delta = now - lastBeat;
      float bpm = 60000.0f / (float)delta;
      if (bpm > 20.0f && bpm < 255.0f) {
        Serial.print("Beat  BPM: "); Serial.print(bpm, 1);
        Serial.print("  IR: "); Serial.println(irValue);
        bpmAccum += bpm;
        beatCount++;
      }
    }
    lastBeat = now;
  }

  // print 2-second window average
  if (millis() - windowStart >= 2000) {
    Serial.print("--- 2s avg: ");
    if (beatCount > 0) {
      Serial.print(bpmAccum / beatCount, 1);
      Serial.println(" BPM");
    } else {
      Serial.println("no valid beats");
    }
    bpmAccum    = 0.0f;
    beatCount   = 0;
    lastBeat    = 0;
    windowStart = millis();
  }
}
