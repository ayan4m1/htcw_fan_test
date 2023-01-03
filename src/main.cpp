#include <Arduino.h>
#include <FastLED.h>

#include "PWMFan.hpp"

PWMFan fan(2, 15, 400);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Initializing");
  fan.begin();
  auto pids = PIDs();
  pids.Kp = 0.4;
  pids.Ki = 0.3;
  pids.Kd = 0.05;
  fan.setPIDs(pids);
  Serial.println("Initialized");
  fan.setTargetRpm(500);
  // fan.tune(1700, 500000);
}

void loop() {
  EVERY_N_MILLIS(500) { fan.update(); }

  EVERY_N_MILLIS(1000) { Serial.printf("RPM: %d\n", fan.getCurrentRpm()); }
}
