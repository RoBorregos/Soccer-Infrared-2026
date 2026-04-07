#include "HeadingPD.h"
#include <Arduino.h>

HeadingPD::HeadingPD(float kp, float kd, float maxTurnPwm, float minTurnPwm, float settleBandDeg)
        : pid_(kp, 0.0, kd, maxTurnPwm),
            lastHeadingError_(0.0),
            lastControlTimeMs_(millis()),
            lastDebugTimeMs_(0),
            maxTurnPwm_(maxTurnPwm),
            minTurnPwm_(minTurnPwm),
            settleBandDeg_(settleBandDeg)
            {}

double HeadingPD::Update(double targetYaw, double currentYaw)
{
    unsigned long now = millis();
    double deltaTime = (now - lastControlTimeMs_) / 1000.0;
    if (deltaTime <= 0.0) deltaTime = 0.001;

    double headingError = wrap(targetYaw - currentYaw, -180.0, 180.0);
    // Use the internal PID to compute P and D (KI set to 0)
    // We feed the PID with setpoint 0 and input -headingError so
    // PID::Calculate computes error = 0 - (-headingError) = headingError
    double turnCommand = pid_.Calculate(0.0, -headingError);

    turnCommand = constrain(turnCommand, -maxTurnPwm_, maxTurnPwm_);

    if (fabs(headingError) <= settleBandDeg_) {
        turnCommand = 0.0;
    } else if (fabs(turnCommand) < minTurnPwm_) {
        turnCommand = (turnCommand >= 0.0) ? minTurnPwm_ : -minTurnPwm_;
    }

    lastHeadingError_ = headingError;
    lastControlTimeMs_ = now;

    return turnCommand;
}
