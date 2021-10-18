#include "ESPNow.h"
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  static ESPNow mySTREAM("YOUR_PREFER_SSID", ESP_NOW_SLAVE);
  while (mySTREAM.available()) {
    Serial.print((char)mySTREAM.read());
  }
}
