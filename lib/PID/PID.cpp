#include "PID.h"

static double wrapRange(double value, double minValue, double maxValue) {
    const double range = maxValue - minValue;
    if (range <= 0.0) {
        return value;
    }

    while (value < minValue) {
        value += range;
    }
    while (value > maxValue) {
        value -= range;
    }
    return value;
}

PID::PID(double kp, double ki, double kd, double minOut, double maxOut, double settleBand)
    : kp_(kp), ki_(ki), kd_(kd), minOut_(minOut), maxOut_(maxOut), settleBand_(settleBand),
      lastError_(0), sumError_(0), lastTimeMs_(millis()) {}

double PID::calculate(double setpoint, double input, bool wrap360) {
    unsigned long now = millis();
    double deltaTime = (now - lastTimeMs_) / 1000.0;

    // Guard against division by zero if called too fast
    if (deltaTime <= 0.0) deltaTime = 0.001; 

    double error = setpoint - input;

    // Optional wrapping logic for Heading
    if (wrap360) {
        error = wrapRange(error, -180.0, 180.0);
    }

    // Standard PID Math
    sumError_ += error * deltaTime;
    double deltaError = (error - lastError_) / deltaTime;

    double output = (kp_ * error) + (ki_ * sumError_) + (kd_ * deltaError);

    // Apply "PWM Gap" and "Settle Band" Logic
    double absError = fabs(error);
    
    if (absError <= settleBand_) {
        output = 0.0;
    } else {
        // Constrain to Max PWM
        output = constrain(output, -maxOut_, maxOut_);

        // Apply Minimum PWM floor
        if (fabs(output) < minOut_) {
            output = (output >= 0) ? minOut_ : -minOut_;
        }
    }

    lastError_ = error;
    lastTimeMs_ = now;
    return output;
}

void PID::reset() {
    sumError_ = 0;
    lastError_ = 0;
    lastTimeMs_ = millis();
}
