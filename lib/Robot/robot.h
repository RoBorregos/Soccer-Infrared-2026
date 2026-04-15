#pragma once
#include <Arduino.h>
#include "constants.h"
#include "motors.h"
#include "BNO.h"
#include "motor.h"
#include "IRRing.h"
#include "photo.h"

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
