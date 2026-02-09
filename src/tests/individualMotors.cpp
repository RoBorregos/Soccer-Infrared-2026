#include "robot.h"
Robot robot;

void setup(){
    // MOVE INDIVIDUAL MOTORS
    robot.left.setSpeed(100);
    delay(1000);
    robot.left.stop();
    delay(500);

    robot.center.setSpeed(100);
    delay(1000);
    robot.center.stop();
    delay(500);

    robot.right.setSpeed(100);
    delay(1000);
    robot.right.stop();
    delay(500); 
}

void loop() {
    
}