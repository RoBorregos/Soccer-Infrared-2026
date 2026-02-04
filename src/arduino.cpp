#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "BNO.h"
#include "constants.h"
#include "motors.h"

Motors motors;
Bno bno;

void setup() {
    Serial.begin(9600);
    motors.begin();
    bno.begin();
    Serial.println("Setup complete");
    // delay(5000);
}

void loop() {
    int base_speed = 65;
    motors.move(0, base_speed);
    delay(650);
    motors.move(90, base_speed);
    delay(650);
    motors.move(180, base_speed);
    delay(650);
    motors.move(270, base_speed);
    delay(650);
    motors.stop();
    for(;;);
}