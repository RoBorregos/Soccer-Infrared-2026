#include <Arduino.h>
#include "constants.h"
#include "robot.h"

Robot bnoTest;

void setup() {
    Serial.begin(9600);
    bnoTest.begin();
    Serial.println("Setup complete");
}

void loop() {
    //double yaw = bnoTest.bno.GetBNOData();
    //Serial.print("Yaw: ");
    //Serial.println(yaw);
}