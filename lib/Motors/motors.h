#pragma once
#include <Arduino.h>
#include <ArduinoSTL.h>
#include "constants.h"
#include "../Motor/motor.h"

class Motors {
public:
    Motors();

    void begin();
    void stop();
    void move(float angleDegrees, float speed, float rotationalSpeed = 0);

private:
    Motor left;
    Motor center;
    Motor right;
};  