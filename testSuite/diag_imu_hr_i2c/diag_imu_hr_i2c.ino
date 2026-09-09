/*
 * diag_imu_hr_i2c.ino
 *
 * Hardware required: LSM6DSO32 + MAX30105 wired to shared I2C bus.
 *
 * Reads both sensors simultaneously and prints to Serial.
 * Beat detection uses check()/available()/nextSample() to step through fresh
 * FIFO samples only — avoids passing stale repeated values to checkForBeat().
 * BPM averaging mirrors the 2-second window logic from sepsorMain.
 *
 * IMU x/y/z printed every 100 ms.
 * BPM window average printed every 2 s.
 * "NO FINGER" printed if IR < 50000.
 */

#include <Adafruit_LSM6DSO32.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

Adafruit_LSM6DSO32 imu;
MAX30105           particleSensor;

// HR accumulation state — mirrors sepsorMain globals
unsigned long lastBeat  = 0;
float         bpmAccum  = 0.0f;
uint8_t       beatCount = 0;

// Diagnostic timers
unsigned long lastImuPrint = 0;
unsigned long windowStart  = 0;

long lastIR = 0; // most recent IR reading for IMU print line

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== diag_imu_hr_i2c ===");

  if (!imu.begin_I2C(0x6A)) {
    Serial.println("ERROR: LSM6DSO32 not found at 0x6A. Check wiring.");
    while (1);
  }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
  Serial.println("IMU OK");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 not found. Check wiring.");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x4F); // higher power for reliable detection
  particleSensor.setPulseAmplitudeIR(0x4F);
  Serial.println("HR sensor OK");

  windowStart = millis();
  Serial.println("\nIMU every 100 ms. BPM window every 2 s. IR > 50000 = finger detected.\n");
}

void loop() {
  // Pull all available FIFO samples and run beat detection on each fresh value.
  // Calling check() first ensures getIR() returns genuinely new data.
  particleSensor.check();
  while (particleSensor.available()) {
    long irValue = particleSensor.getFIFOIR();
    lastIR = irValue;
    particleSensor.nextSample();

    if (irValue < 50000) continue; // no finger — skip beat detection

    if (checkForBeat(irValue)) {
      unsigned long now = millis();
      if (lastBeat != 0) {
        unsigned long delta = now - lastBeat;
        float bpm = 60000.0 / (float)delta;
        if (bpm > 20.0 && bpm < 255.0) {
          bpmAccum += bpm;
          beatCount++;
        }
      }
      lastBeat = now;
    }
  }

  // IMU print every 100 ms
  if (millis() - lastImuPrint >= 100) {
    lastImuPrint = millis();
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    Serial.print("AX: "); Serial.print(accel.acceleration.x, 3);
    Serial.print("  AY: "); Serial.print(accel.acceleration.y, 3);
    Serial.print("  AZ: "); Serial.print(accel.acceleration.z, 3);
    Serial.print("  m/s²  |  IR: "); Serial.print(lastIR);
    Serial.println(lastIR < 50000 ? "  [NO FINGER]" : "");
  }

  // 2-second BPM window commit — mirrors sepsorMain readHR block
  if (millis() - windowStart >= 2000) {
    Serial.print("--- 2s BPM avg: ");
    if (beatCount > 0) {
      Serial.print(bpmAccum / beatCount, 1);
      Serial.println(" BPM");
    } else {
      Serial.println(lastIR < 50000 ? "no finger" : "no beats detected");
    }
    delay(1000);
    bpmAccum    = 0.0f;
    beatCount   = 0;
    lastBeat    = 0;
    windowStart = millis();
  }
}
