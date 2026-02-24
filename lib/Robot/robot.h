#pragma once
#include <Arduino.h>
#include "motors.h"
#include "BNO.h"
#include "motor.h"

class Robot {
public:
    void begin();

    Motors motors;
    Bno bno;
    Motor motor;
};