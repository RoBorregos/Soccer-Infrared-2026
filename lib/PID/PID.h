#pragma once
#include <Arduino.h>

class PIDParameters {
public:
    double kp;
    double ki;
    double kd;

    double max_output = 0.0;
    double min_output = 0.0;
    float error_threshold = 0.0;

    double integral_error = 0.0;
    double previous_error = 0.0;
    unsigned long long last_iteration_ms = 0;

    double target = 0.0;
    double error = 0.0;
    double output = 0.0;

    double current_value = 0.0;
    bool first_run = true;

    PIDParameters(double kp, double ki, double kd, double max_output, double min_output, float error_threshold) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->max_output = max_output;
        this->min_output = min_output;
        this->error_threshold = error_threshold;
    }
};

class PID {
public:
    PID(PIDParameters parameters) : parameters(parameters) {};
    static void reset(PIDParameters& parameters );
    static double calculate(PIDParameters& parameters);

private:
    PIDParameters parameters;
};