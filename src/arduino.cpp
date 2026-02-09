#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "BNO.h"
#include "constants.h"
#include "motors.h"

// DEBUG 
#include "motorsTest.cpp"

Motors motors;
Bno bno;

Debug debug;

void setup() {
    Serial.begin(9600);
    motors.begin();
    bno.begin();
    Serial.println("Setup complete");
    delay(5000);
}

void loop() {
    debug.motorSquare();
    delay(1000);
    debug.motorMoveIndividualMotors();

    for(;;);
}