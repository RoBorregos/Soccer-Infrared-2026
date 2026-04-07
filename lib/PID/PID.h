#pragma once
#include <Arduino.h>

class PID {
public:
    // Combined constructor: includes standard PID gains plus the PWM constraints
    PID(double kp, double ki, double kd, double minOut, double maxOut, double settleBand);

    // Main calculation method
    // Added 'wrap' parameter to handle heading logic only when needed
    double calculate(double setpoint, double input, bool wrap360 = false);

    // Helper to reset integral/timing if the target changes significantly
    void reset();

private:
    double kp_, ki_, kd_;
    double minOut_, maxOut_, settleBand_;
    
    double lastError_;
    double sumError_;
    unsigned long lastTimeMs_;

    // Internal angle wrapping logic from HeadingPID
    double wrapValue(double value, double min, double max);
};