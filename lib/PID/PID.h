#pragma once
#include <Arduino.h>
#include "constants.h"

class PID {
public:
    PID(double kp, double ki, double kd, double minOut, double maxOut, double settleBand);

    double calculate(double setpoint, double input, bool wrap360 = false);
    void reset();

private:
    double kp_, ki_, kd_;
    double minOut_, maxOut_, settleBand_;
    
    double lastError_;
    double sumError_;
    unsigned long lastTimeMs_;
};