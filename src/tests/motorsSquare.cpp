#include "robot.h"

void setup(){
    
}

void loop() {
    // MOVE AS A SQUARE
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
    delay(1000);
}