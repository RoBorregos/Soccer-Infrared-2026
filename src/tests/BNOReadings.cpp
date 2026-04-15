#include <Arduino.h>
#include <Wire.h>
#include "BNO.h"

Bno bnoTest;
unsigned long lastPrintTime = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);
    bnoTest.begin();
    Serial.println("Setup complete");
}

void loop() {
    const unsigned long now = millis();
    if (now - lastPrintTime < 50) {
        return;
    }

    lastPrintTime = now;
    const double yaw = bnoTest.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(yaw);
}
