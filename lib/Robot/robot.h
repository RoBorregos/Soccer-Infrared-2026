#pragma once
#include <Arduino.h>
#include "..\Motors\motors.h"
#include "..\BNO\BNO.h"

extern Motors motors;
extern Bno bno;

class Robot {
public:
    Robot();
    void begin();
};