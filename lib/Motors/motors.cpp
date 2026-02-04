
#include "motors.h"

Motors::Motors() {
    motor1.pwmPin = constants::kMotor1Pwm;
    motor1.in1Pin = constants::kMotor1In1;
    motor1.in2Pin = constants::kMotor1In2;

    motor2.pwmPin = constants::kMotor2Pwm;
    motor2.in1Pin = constants::kMotor2In1;
    motor2.in2Pin = constants::kMotor2In2;

    motor3.pwmPin = constants::kMotor3Pwm;
    motor3.in1Pin = constants::kMotor3In1;
    motor3.in2Pin = constants::kMotor3In2;
}

void Motors::begin() {
    motor1.begin();
    motor2.begin();
    motor3.begin();
}

void Motors::stop() {
    motor1.stop();
    motor2.stop();
    motor3.stop();
}

void Motors::move(float angleDegrees, float speedPercent, float rotationalSpeed) {
    float upper_left_speed = cos((angleDegrees - 150) * PI / 180.0f) * speedPercent + rotationalSpeed;
    float lower_center_speed = cos((angleDegrees - 270) * PI / 180.0f) * speedPercent + rotationalSpeed;
    float upper_right_speed = cos((angleDegrees - 30) * PI / 180.0f) * speedPercent + rotationalSpeed;

    motor1.setSpeed(upper_left_speed);
    motor2.setSpeed(lower_center_speed);
    motor3.setSpeed(upper_right_speed);
}

void Motors::Motor::begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    stop();
}

void Motors::Motor::setSpeed(float speed) {
    speed = constrain(speed, -255.0f, 255.0f);

    if (speed > 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
    analogWrite(pwmPin, floor(abs(speed)));
}

void Motors::Motor::stop() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    setSpeed(0);
}