#include <Arduino.h>
#include "constants.h"
#include "robot.h"

Robot robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
    Serial.println("Setup complete");
}

void loop() {
    double yaw = bno.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(yaw);
}