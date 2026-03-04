#include "robot.h"

Robot::Robot() {
    currentTime = millis();
}

void Robot::begin() {
    motors.begin();
    bno.begin();
    irring.init(&currentTime);
}
