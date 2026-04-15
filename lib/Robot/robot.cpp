#include "robot.h"

Robot::Robot() {
    currentTime = millis();
}

void Robot::begin() {
    // Motors
    motors.begin();

    // BNO085
    Wire.begin();
    Wire.setClock(400000);
    bno.begin();

    // IR Ring
    irring.init(&currentTime);
}
