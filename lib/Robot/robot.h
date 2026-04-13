#pragma once
#include <Arduino.h>
#include "constants.h"
#include "motors.h"
// #include "BNO.h"
#include "motor.h"
#include "IRRing.h"
#include "IMU.h"
#include "photo.h"

class Robot {
public:
    Robot();
    void begin();

    Motors motors;
    // Bno bno;
    IRRing irring;
    IMUDriver imu;
private:
    unsigned long currentTime;
};
