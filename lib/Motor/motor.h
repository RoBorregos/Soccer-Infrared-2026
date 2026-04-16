#pragma once
#include <Arduino.h>
#include "constants.h"

class Motor {
public:
    Motor(int id, int pwmPin, int in1Pin, int in2Pin);
    void begin();
    void setSpeed(float speed);
    void stop();

private:
    int id;
    int pwmPin;
    int in1Pin;
    int in2Pin;
};
