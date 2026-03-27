#pragma once
#include <Arduino.h>
#include "motors.h"
#include "BNO.h"
#include "motor.h"
#include "IRRing.h"

class Robot {
public:
    Robot();
    void begin();

    Motors motors;
    Bno bno;
    IRRing irring;

private:
    unsigned long currentTime;
};