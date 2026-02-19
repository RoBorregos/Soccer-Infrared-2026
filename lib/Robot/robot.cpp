#include "Robot.h"

Motors motors;
Bno bno;

Robot::Robot() {
}

void Robot::begin() {
    motors.begin();
    bno.begin();
}
