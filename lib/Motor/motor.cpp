#include "motor.h"

namespace
{
float GetMotorSpeedOffset(int motorId)
{
    switch (motorId)
    {
    case 1:
        return Constants::Motor::Left::speedOffset;
    case 2:
        return Constants::Motor::Center::speedOffset;
    case 3:
        return Constants::Motor::Right::speedOffset;
    default:
        return 0.0f;
    }
}

float GetMotorDirectionSign(int motorId)
{
    switch (motorId)
    {
    case 1:
        return Constants::Motor::Left::directionSign;
    case 2:
        return Constants::Motor::Center::directionSign;
    case 3:
        return Constants::Motor::Right::directionSign;
    default:
        return 1.0f;
    }
}
}

Motor::Motor(int id, int pwmPin, int in1Pin, int in2Pin) {
    this->id = id;
    this->pwmPin = pwmPin;
    this->in1Pin = in1Pin;
    this->in2Pin = in2Pin;
}

void Motor::begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    stop();
}

void Motor::setSpeed(float speed) {
    speed *= GetMotorDirectionSign(id);

    if (speed > 0.0f) {
        speed += GetMotorSpeedOffset(id);
    } else if (speed < 0.0f) {
        speed -= GetMotorSpeedOffset(id);
    }

    speed = constrain(speed, -255.0f, 255.0f);

    // Don't touch direction pins if speed is zero
    if (abs(speed) < 1.0f) {
        analogWrite(pwmPin, 0);
        return;
    }

    if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }

    analogWrite(pwmPin, (int)abs(speed));
}

void Motor::stop() {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
}
