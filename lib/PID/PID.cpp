#include "PID.h"

void PID::reset(PIDParameters& pid) {
    pid.integral_error = 0.0;
    pid.previous_error = 0.0;
    pid.output = 0.0;
    pid.error = 0.0;
    pid.target = 0.0;
    pid.last_iteration_ms = 0;
    pid.first_run = true;
}

double PID::calculate(PIDParameters& pid) {
    unsigned long long current_time_ms = millis();

    pid.error = pid.target - pid.current_value;
    // Serial.print(" | C: ");
    // Serial.print(pid.current_value);
    // Serial.print(" | E: ");
    Serial.print(pid.error);
    if (pid.first_run) {
        pid.last_iteration_ms = current_time_ms;
        pid.previous_error = pid.error;
        pid.integral_error = 0.0;
        pid.first_run = false;

        double pTerm = pid.kp * pid.error;
        double output = pTerm;

        if (pid.max_output > 0.0) {
            output = constrain(output, -pid.max_output, pid.max_output);
        }
        if (pid.min_output > 0.0 && fabs(output) > 0.0 && fabs(output) < pid.min_output) {
            output = (output > 0.0) ? pid.min_output : -pid.min_output;
        }

        Serial.print("EXECUTED ONCE");
        pid.output = output;
        return pid.output;
    }

    double dt = (current_time_ms - pid.last_iteration_ms) / 1000.0;
    Serial.print("dt: ");
    Serial.println(dt, 5);
    if (dt <= 0.0) {
        return pid.output;
    }

    if (fabs(pid.error) < pid.error_threshold) {
        pid.output = 0.0;
        return pid.output;
    }
    double pTerm = pid.kp * pid.error;
    double dTerm = pid.kd * (pid.error - pid.previous_error) / dt;
    // Tentative integral for anti-windup
    double newIntegral = pid.integral_error + pid.error * dt;
    double iTerm = pid.ki * newIntegral;

    Serial.print("P: ");
    Serial.print(pTerm, 5);
    Serial.print(" | I: ");
    Serial.print(iTerm, 5);
    Serial.print(" | D: ");
    Serial.print(dTerm, 5);
    Serial.print(" | O: ");
    Serial.println(pTerm + iTerm + dTerm, 5);

    double output = pTerm + iTerm + dTerm;

    // Anti-windup & clamping
    if (pid.max_output > 0.0) {
        if (output > pid.max_output) {
            output = pid.max_output;
            newIntegral = pid.integral_error;
        } else if (output < -pid.max_output) {
            output = -pid.max_output;
            newIntegral = pid.integral_error;
        }
    }

    // Ensure at least min_output magnitude if non-zero, otherwise freeze integral
    if (pid.min_output > 0.0 && fabs(output) > 0.0 && fabs(output) < pid.min_output) {
        output = (output > 0.0) ? pid.min_output : -pid.min_output;
        newIntegral = pid.integral_error;
    }

    // Commit state
    pid.integral_error = newIntegral;
    pid.previous_error = pid.error;
    pid.last_iteration_ms = current_time_ms;
    pid.output = output;

    return output;
}