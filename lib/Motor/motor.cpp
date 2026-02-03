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

/**
 * @brief Set the speed of the motor as a percentage.
 * @param speedPercent Speed percentage (0 to 100).
 * 
 * Its just a wrapper of SetPWM with a static cast from float to int.
 */
void Motor::SetSpeed(float speedPercent) {
    SetPWM(static_cast<int>(speedPercent/100 * 255));
}

/**
 * @brief Move the motor in the specified direction at the given speed percentage.
 * @param speedPercent Speed percentage (0 to 100).
 * @param direction Direction of the motor movement (FORWARD or BACKWARD).
 * 
 * FORWARD is designed for the motors to move clockwise.
 * BACKWARD is designed for the motors to move counter-clockwise.
 * 
 * If the direction is BACKWARD, the speed percentage is negated to ensure correct motor behavior; 
 * rather than changing the sign of speedPercent directly, we set the speed to its negative value.
 */
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


/**
 * @brief Stop the motor by setting both control pins to LOW and speed to 0.
 * 
 * This function ensures that the motor halts its movement immediately.
 */
void Motor::StopMotor() {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
    SetSpeed(0);
}