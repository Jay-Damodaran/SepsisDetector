/*
 * diag_battery.ino
 *
 * Hardware required: LiPo battery + voltage divider wired to A0.
 *
 * Prints battery status to Serial every second:
 *   - Raw ADC reading (half-rail mV, after voltage divider)
 *   - Actual battery voltage (×2 to undo divider)
 *   - Estimated state of charge (%)
 *   - OK / LOW BATTERY status using the same threshold as sepsorMain
 *
 * Also runs lowBatteryDetect() verbatim so you can verify the hysteresis
 * logic (charging recovery detection) matches sepsorMain behavior.
 *
 * LiPo range: 3.1 V (empty) to 4.2 V (full).
 * Voltage divider halves the voltage → 1550–2100 mV at the ADC pin.
 * 20% threshold ≈ 1660 mV half-rail (3.32 V actual).
 */

#define BATTERY    A0
#define MAX_V      2100   // mV half-rail at 4.2 V
#define MIN_V      1550   // mV half-rail at 3.1 V
#define RECOVERY_V 100    // hysteresis: ignore recovery < 0.1 V half-rail

const int v_threshold = (int)(MIN_V + 0.2f * (MAX_V - MIN_V)); // ~1660 mV

uint32_t      warning_val = 0;
unsigned long lastPrint   = 0;

// Verbatim from sepsorMain
int lowBatteryDetect() {
  uint32_t val = 0;
  for (uint8_t i = 0; i < 5; i++) val += analogReadMilliVolts(BATTERY);
  val /= 5;

  if (val >= (uint32_t)v_threshold) { warning_val = 0; return 0; }
  if (!warning_val)                 { warning_val = val; return 1; }
  if (val > warning_val && (val - warning_val) > RECOVERY_V) return 0;
  return 1;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== diag_battery ===");
  Serial.print("20% threshold: ");
  Serial.print(v_threshold * 2);
  Serial.println(" mV actual  (");
  Serial.print(v_threshold);
  Serial.println(" mV half-rail)");
  Serial.println();
  Serial.println("Half-rail(mV) | Actual(mV) | SoC(%) | Status");
  Serial.println("----------------------------------------------");
}

void loop() {
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  uint32_t raw = 0;
  for (uint8_t i = 0; i < 5; i++) raw += analogReadMilliVolts(BATTERY);
  raw /= 5;

  uint32_t actual_mv = raw * 2;
  int pct = constrain(
    (int)(100.0f * ((float)raw - MIN_V) / (MAX_V - MIN_V)),
    0, 100
  );
  int low = lowBatteryDetect();

  Serial.print(raw);
  Serial.print("          | ");
  Serial.print(actual_mv);
  Serial.print("       | ");
  Serial.print(pct);
  Serial.print("     | ");
  Serial.println(low ? "LOW BATTERY" : "OK");
}
