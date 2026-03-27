#include "robot.h"

Robot squareTest;
void setup(){
    squareTest.begin();
}

void loop() {
    // MOVE AS A SQUARE
    int base_speed = 100;
    squareTest.motors.move(0, base_speed);
    delay(650);
    squareTest.motors.move(90, base_speed);
    delay(650);
    squareTest.motors.move(180, base_speed);
    delay(650);
    squareTest.motors.move(270, base_speed);
    delay(650);
    squareTest.motors.stop();
    delay(1000);
}