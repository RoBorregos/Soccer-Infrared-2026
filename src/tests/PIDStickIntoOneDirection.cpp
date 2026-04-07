#include <Arduino.h>
#include "constants.h"
#include "robot.h"

Robot robot;

double targetYaw = 0.0;
double lastHeadingError = 0.0;
unsigned long lastControlTimeMs = 0;
unsigned long lastDebugTimeMs = 0;

const float kHeadingKp = 2.3f;
const float kHeadingKd = 0.18f;
const float kMaxTurnPwm = 90.0f;
const float kMinTurnPwm = 45.0f;
const float kHeadingSettleBandDeg = 4.0f;
const unsigned long kDebugIntervalMs = 100;

static double WrapAngle180(double deg) {
    while (deg > 180.0) {
        deg -= 360.0;
    }
    while (deg < -180.0) {
        deg += 360.0;
    }
    return deg;
}

static double ClampMagnitude(double value, double maxMagnitude) {
    if (value > maxMagnitude) {
        return maxMagnitude;
    }
    if (value < -maxMagnitude) {
        return -maxMagnitude;
    }
    return value;
}

void setup() {
    Serial.begin(115200);
    robot.begin();
    delay(2000);

    targetYaw = robot.imu.getAngle();
    lastControlTimeMs = millis();
}

void loop() {
    double yaw = robot.imu.getAngle();
    unsigned long now = millis();
    double deltaTime = (now - lastControlTimeMs) / 1000.0;
    if (deltaTime <= 0.0) {
        deltaTime = 0.001;
    }

    double headingError = WrapAngle180(targetYaw - yaw);
    double headingDerivative = (headingError - lastHeadingError) / deltaTime;

    double turnCommand = (kHeadingKp * headingError) + (kHeadingKd * headingDerivative);
    turnCommand = ClampMagnitude(turnCommand, kMaxTurnPwm);

    if (fabs(headingError) <= kHeadingSettleBandDeg) {
        turnCommand = 0.0;
    } else if (fabs(turnCommand) < kMinTurnPwm) {
        turnCommand = (turnCommand >= 0.0) ? kMinTurnPwm : -kMinTurnPwm;
    }

    if (now - lastDebugTimeMs >= kDebugIntervalMs) {
        Serial.print("Target: ");
        Serial.print(targetYaw);
        Serial.print(" Yaw: ");
        Serial.print(yaw);
        Serial.print(" Error: ");
        Serial.print(headingError);
        Serial.print(" Turn: ");
        Serial.println(turnCommand);
        lastDebugTimeMs = now;
    }

    lastHeadingError = headingError;
    lastControlTimeMs = now;

    robot.motors.move(0, 0, turnCommand);

}
