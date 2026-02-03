#include <Arduino.h>
#include "constants.h"
#include "motor.h"


Motor::Motor(int pwmPin, int in1Pin, int in2Pin){
    pwmPin_ = pwmPin;
    in1Pin_ = in1Pin;
    in2Pin_ = in2Pin;
}

void Motor::InitializeMotor() {
    pinMode(pwmPin_, OUTPUT);
    pinMode(in1Pin_, OUTPUT);
    pinMode(in2Pin_, OUTPUT);
    StopMotor();
}

void Motor::SetPWM(const uint8_t pwm)
{
    analogWrite(pwmPin_, pwm);
};

void Motor::SetSpeed(float speedPercent) {
    SetPWM(static_cast<int>(speedPercent * 255));
}

void Motor::MoveMotor(float speedPercent, bool direction) {
    if (direction == BACKWARD) {
        digitalWrite(in1Pin_, LOW);
        digitalWrite(in2Pin_, HIGH);
        SetSpeed(-speedPercent);
    } else if (direction == FORWARD) {
    digitalWrite(in1Pin_, HIGH);
    digitalWrite(in2Pin_, LOW);
    SetSpeed(speedPercent);
    }
}


void Motor::StopMotor() {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
    SetSpeed(0);
}