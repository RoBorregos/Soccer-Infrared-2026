#pragma once
#include <Arduino.h>
#include "motors.h"
#include "BNO.h"

extern Motors motors;
extern Bno bno;

class Robot {
public:
    Robot();
    void begin();
};