#include "motors.h"

extern Motors motors;

class Debug {
public:
    void motorSquare() {
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
    }

    void motorMoveIndividualMotors() {
        motors.left.setSpeed(50);
        delay(1000);
        motors.left.stop();
        motors.center.setSpeed(50);
        delay(1000);
        motors.center.stop();
        motors.right.setSpeed(50);
        delay(1000);
        motors.right.stop();
    }
};