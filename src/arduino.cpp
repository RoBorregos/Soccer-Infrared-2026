#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "constants.h"
#include "robot.h"

Motors motors;
Bno bno;

void setup() {
    Serial.begin(9600);
    motors.begin();
    bno.begin();
    Serial.println("Setup complete");
    delay(5000);
}

void loop() {
    for(;;);
}