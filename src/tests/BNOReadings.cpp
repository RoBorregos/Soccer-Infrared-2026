#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"

Bno bno;

void setup() {
    Serial.begin(9600);
    bno.begin();
    Serial.println("Setup complete");
}

void loop() {
    double yaw = bno.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    double targetYaw = 0.0;
}