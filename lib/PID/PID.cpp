#include "PID.h"

void PID::reset(PIDParameters& pid) {
    pid.integral_error = 0;
    pid.previous_error = 0;
    pid.output = 0;
    pid.error = 0;
    pid.target = 0;
    pid.last_iteration_ms = 0;
    pid.first_run = true;
}

double PID::calculate(PIDParameters& pid) {
    unsigned long long current_time_ms = millis();
    
    // FROM AIZER PID IMPLEMENTATION
    if (pid.first_run) {
        pid.last_iteration_ms = current_time_ms;
        pid.previous_error = pid.error;
        pid.integral_error = 0.0;
        pid.first_run = false;

        const double pTerm = pid.kp * pid.error;
        double initial_output = pTerm;

        if (pid.min_output > 0.0 && fabs(initial_output) < pid.min_output) {
            initial_output = 0.0;
        }

        if (pid.max_output > 0.0) {
            initial_output = constrain(
                initial_output,
                -pid.max_output,
                pid.max_output
            );
        }

        pid.output = initial_output;
        return pid.output;
    }

    double dt = (current_time_ms - pid.last_iteration_ms) / 1000.0; 
    double previous_integral = pid.integral_error;
    pid.error = pid.target - pid.current_value;


    if (dt <= 0.0) return pid.output; 
    if (abs(pid.error) < pid.error_threshold) {
        return 0.0; 
    }

    pid.integral_error += pid.error * dt;

    double derivative_error = (pid.error - pid.previous_error) / dt;
    double output = (pid.kp * pid.error) + (pid.ki * pid.integral_error) + (pid.kd * derivative_error);

    if(output > pid.max_output && output < 0) {
        pid.current_value = constrain(pid.current_value, pid.min_output, pid.max_output);
        pid.integral_error = previous_integral;

    } else if (output < pid.min_output && output > 0) {
        pid.current_value = constrain(pid.current_value, -pid.max_output, -pid.min_output);
        pid.integral_error = previous_integral;
    }

    pid.previous_error = pid.error;
    pid.last_iteration_ms = current_time_ms;
    pid.output = output;

    return output;
}