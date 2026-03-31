#include <Arduino.h>
#include "IMU.h"

// Instantiate wrapper and use it in example setup/loop
IMUDriver imuDriver(&IMU, IMU_ADDRESS);

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    while (!Serial) {
        ;
    }

    int err = imuDriver.begin(calib);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true) {
          ;
        }
    }

    delay(1000);
}

void loop() {
  // Query angle at a regular interval
  static unsigned long last_ms = 0;
  if (millis() - last_ms >= 25) {
    float angle = imuDriver.getAngle();
    Serial.print("Yaw: ");
    Serial.println(angle);
    last_ms = millis();
  }
}