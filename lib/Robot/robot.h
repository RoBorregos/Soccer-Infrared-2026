#pragma once
#include <Arduino.h>
#include "constants.h"
#include "motors.h"
// #include "BNO.h"
#include "motor.h"
#include "IRRing.h"
#include "IMU.h"
#include "Pixy.h"
#include "photo.h"

class Robot {
public:
    Robot();
    void begin();

    Motors motors;
    // Bno bno;
    IRRing irring;
    IMUDriver imu;
    Pixy2 pixy;
private:
    unsigned long currentTime;
};