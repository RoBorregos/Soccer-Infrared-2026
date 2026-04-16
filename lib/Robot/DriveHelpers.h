#pragma once

#include <Arduino.h>

namespace DriveHelpers {

inline float clampSymmetric(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

inline float smoothAngleChange(float previous, float current, float alpha) {
    return previous + (current - previous) * alpha;
}

inline float wrapAngle180(float angleDegrees) {
    while (angleDegrees > 180.0f) {
        angleDegrees -= 360.0f;
    }
    while (angleDegrees < -180.0f) {
        angleDegrees += 360.0f;
    }
    return angleDegrees;
}

}  // namespace DriveHelpers
