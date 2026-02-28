#include "robot.h"
Robot robot;

void setup(){
    // MOVE AS A SQUARE
    robot.begin();
    delay(3000);
    int base_speed = 65;
    robot.move(0, base_speed);
    // delay(650);
    // robot.move(90, base_speed);
    // delay(650);
    // robot.move(180, base_speed);
    // delay(650);
    // robot.move(270, base_speed);
    // delay(650);
    // robot.stop();
    // delay(1000);
}

void loop() {
    
}