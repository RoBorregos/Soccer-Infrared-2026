#include <Arduino.h>
#include "BNO.h"

Bno bnoTest;

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    while (!Serial) {
        ;
    }

    bnoTest.begin();
    delay(1000);
}

void loop() {
  static unsigned long last_ms = 0;
  if (millis() - last_ms >= 25) {
    const double angle = bnoTest.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(angle);
    last_ms = millis();
  }
}
