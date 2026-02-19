#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "constants.h"
#include "robot.h"

Robot robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
    Serial.println("Setup complete");
    delay(5000);
}

void loop() {
    for(;;);
}