#include <Arduino.h>
#include "constants.h"
#include "motor.h"
#include "motors.h"

typedef void (Motor::*NoArgAction)();
typedef void (Motor::*MoveAction)(float, bool);

static Motor* motors[3] = { nullptr, nullptr, nullptr };

static void forEach(NoArgAction action) {
    for (int i = 0; i < 3; ++i) {
        if (motors[i]) {
            (motors[i]->*action)();
        }
    }
}

static void forEachMove(MoveAction action, float speedPercent, bool direction) {
    for (int i = 0; i < 3; ++i) {
        if (motors[i]) {
            (motors[i]->*action)(speedPercent, direction);
        }
    }
}

static void forEachMoveExcept(MoveAction action, float speedPercent, bool direction, int exceptIndex) {
    for (int i = 0; i < 3; ++i) {
        if (i != exceptIndex && motors[i]) {
            (motors[i]->*action)(speedPercent, direction);
        }
    }
}

Motors::Motors(int kMotor1Pwm, int kMotor1In1, int kMotor1In2,
        int kMotor2Pwm, int kMotor2In1, int kMotor2In2,
        int kMotor3Pwm, int kMotor3In1, int kMotor3In2) {
            motors[0] = new Motor(kMotor1Pwm, kMotor1In1, kMotor1In2);
            motors[1] = new Motor(kMotor2Pwm, kMotor2In1, kMotor2In2);
            motors[2] = new Motor(kMotor3Pwm, kMotor3In1, kMotor3In2);
}

void Motors::InitializeMotor() {
    forEach(&Motor::InitializeMotor);
}

void Motors::RotateRobot(float speedPercent, bool direction) {
    forEachMove(&Motor::MoveMotor, speedPercent, direction);
}

void Motors::MoveRobot(float speedPercent, bool direction, int exceptIndex) {
    forEachMoveExcept(&Motor::MoveMotor, speedPercent, direction, exceptIndex);
}

void Motors::StopMotor() {
    forEach(&Motor::StopMotor);
}

void Motors::moveRobotOmnidirectional(float angleDegrees, float speedPercent, float rotationalSpeed) {
    float upper_left_speed = cos((angleDegrees - 150) * PI / 180.0f) * speedPercent + rotationalSpeed;
    float lower_center_speed = cos((angleDegrees - 270) * PI / 180.0f) * speedPercent + rotationalSpeed;
    float upper_right_speed = cos((angleDegrees - 30) * PI / 180.0f) * speedPercent + rotationalSpeed;

    if (motors[0]) motors[0]->SetSpeed(upper_left_speed);
    if (motors[1]) motors[1]->SetSpeed(lower_center_speed);
    if (motors[2]) motors[2]->SetSpeed(upper_right_speed);
}