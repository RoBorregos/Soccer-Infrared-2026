#pragma once
#include <Arduino.h>
#include "constants.h"
#include "PID.h"

class HeadingPD {
public:
    HeadingPD(float kp, float kd, float maxTurnPwm, float minTurnPwm, float settleBandDeg);
    double Update(double targetYaw, double currentYaw);

private:
    PID pid_;
    double lastHeadingError_;
    unsigned long lastControlTimeMs_;
    unsigned long lastDebugTimeMs_;
    float maxTurnPwm_;
    float minTurnPwm_;
    float settleBandDeg_;
    unsigned long debugIntervalMs_;
    bool debug_;
};
