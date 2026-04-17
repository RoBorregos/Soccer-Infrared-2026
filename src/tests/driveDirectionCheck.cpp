#include <Arduino.h>
#include "motors.h"
#include "constants.h"

namespace {

Motors motors;

const float kDrivePwm = 0.40f * Constants::Motor::maxPWM;
constexpr float kRotatePwm = 38.0f;
constexpr unsigned long kStepDurationMs = 1500;
constexpr unsigned long kPauseDurationMs = 900;

struct MotionStep {
    const char* label;
    float angleDeg;
    float drivePwm;
    float turnPwm;
};

const MotionStep kSteps[] = {
    {"FORWARD  angle=0", 0.0f, kDrivePwm, 0.0f},
    {"BACKWARD angle=180", 180.0f, kDrivePwm, 0.0f},
    {"LEFT     angle=-90", -90.0f, kDrivePwm, 0.0f},
    {"RIGHT    angle=90", 90.0f, kDrivePwm, 0.0f},
    {"ROTATE   clockwise", 0.0f, 0.0f, kRotatePwm},
    {"ROTATE   counterclockwise", 0.0f, 0.0f, -kRotatePwm},
};

void runStep(const MotionStep& step) {
    Serial.println();
    Serial.println(step.label);
    Serial.print("drive=");
    Serial.print(step.drivePwm, 1);
    Serial.print(" turn=");
    Serial.println(step.turnPwm, 1);
    motors.move(step.angleDeg, step.drivePwm, step.turnPwm);
    delay(kStepDurationMs);
    motors.stop();
    delay(kPauseDurationMs);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    motors.begin();
    motors.stop();
    delay(1500);

    Serial.println("Drive direction check ready");
    Serial.println("Watch chassis motion, not just wheel spin direction.");
}

void loop() {
    for (const MotionStep& step : kSteps) {
        runStep(step);
    }
}
