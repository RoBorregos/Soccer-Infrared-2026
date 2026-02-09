#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "BNO.h"
#include "constants.h"
#include "robot.h"

Robot robot;
Bno bno;

void setup() {
    Serial.begin(9600);
    robot.begin();
    bno.begin();
    Serial.println("Setup complete");
    delay(5000);
}

void loop() {
    for(;;);
}