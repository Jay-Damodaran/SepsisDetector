#include "WiFi.h"
#include "esp_bt.h"

#define SLEEP_US   (2ULL * 3600ULL * 1000000ULL) // 2 hours
#define WAKE_MS    (20UL * 60UL * 1000UL)         // 20 minutes

void goToSleep() {
  Serial.println("Sleeping for 2 hours. Hold B + tap R anytime to force flash.");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    // woke from 2-hour timer — stay alive for flashing window
    Serial.println("Wake window open for 20 min. Flash now or wait.");
  } else {
    // fresh boot — shut down immediately
    goToSleep();
  }
}

void loop() {
  if (millis() >= WAKE_MS) {
    goToSleep();
  }
}
