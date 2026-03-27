#include "robot.h"

Robot::Robot()
    : left(1,
            Constants::Motor::Left::pwm,
            Constants::Motor::Left::in1,
            Constants::Motor::Left::in2),
      center(2,
             Constants::Motor::Center::pwm,
             Constants::Motor::Center::in1,
             Constants::Motor::Center::in2),
      right(3,
            Constants::Motor::Right::pwm,
            Constants::Motor::Right::in1,
            Constants::Motor::Right::in2) {
}

void Robot::begin() {
    left.begin();
    center.begin();
    right.begin();
}

void Robot::stop() {
    left.stop();
    center.stop();
    right.stop();
}

void Robot::move(float angleDegrees, float speedPercent, float rotationalSpeed) {
    float upper_left_speed =-( cos((angleDegrees - 150) * PI / 180.0f) * speedPercent + rotationalSpeed);
    float lower_center_speed = cos((angleDegrees - 270) * PI / 180.0f) * speedPercent + rotationalSpeed;
    float upper_right_speed = cos((angleDegrees - 30) * PI / 180.0f) * speedPercent + rotationalSpeed;

    left.setSpeed(upper_left_speed);
    center.setSpeed(lower_center_speed);
    right.setSpeed(upper_right_speed);
}