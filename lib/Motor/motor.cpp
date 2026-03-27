#include "motor.h"

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
    speed = constrain(speed, -255.0f, 255.0f);

    if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }

    int pwm = floor(abs(speed));
    // if (id == 2) pwm -= 15; // Calibration for motor 2
    
    pwm = constrain(pwm, 0, 255);

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" Speed: ");
    Serial.println(pwm);
    analogWrite(pwmPin, pwm);
}

void Motor::stop() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    setSpeed(0);
}