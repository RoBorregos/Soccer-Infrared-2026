#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "robot.h"

#define HWSERIAL Serial3

namespace {

Robot robot;

const float kHeadingKp = 1.5f;
const float kHeadingKd = 0.10f;
const float kMaxTurnPwm = 55.0f;
const float kMinTurnPwm = 12.0f;
const float kHeadingSettleBandDeg = 6.0f;
const float kDrivePwm = 0.45f * Constants::Motor::maxPWM;
const unsigned long kDebugIntervalMs = 100;
const unsigned long kBallTimeoutMs = 250;

PID headingPD(kHeadingKp, 0.0f, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);

double targetYaw = 0.0;
float latestBallAngle = 0.0f;
unsigned long lastBallReadMs = 0;

}

float wrapAngle180(float angleDegrees) {
    while (angleDegrees > 180.0f) {
        angleDegrees -= 360.0f;
    }
    while (angleDegrees < -180.0f) {
        angleDegrees += 360.0f;
    }
    return angleDegrees;
}

void setup() {
    Serial.begin(115200);
    HWSERIAL.begin(9600);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();
}

void loop() {
    static unsigned long lastDebugMs = 0;

    if (HWSERIAL.available() > 0) {
        String incomingData = HWSERIAL.readStringUntil('\n');
        incomingData.trim();

        if (incomingData.length() > 0) {
            if (incomingData.startsWith("a ")) {
                const float incomingAngle = incomingData.substring(2).toFloat();
                latestBallAngle = wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            } else if (!incomingData.startsWith("r ")) {
                const float incomingAngle = incomingData.toFloat();
                latestBallAngle = wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            }
        }
    }

    const double yaw = robot.bno.GetBNOData();
    const double turnCommand = headingPD.calculate(targetYaw, yaw, true);
    const bool hasBall = (millis() - lastBallReadMs) <= kBallTimeoutMs;

    if (hasBall) {
        robot.motors.move(latestBallAngle, kDrivePwm, turnCommand);
    } else {
        robot.motors.move(0.0f, 0.0f, turnCommand);
    }

    if (millis() - lastDebugMs >= kDebugIntervalMs) {
        lastDebugMs = millis();
        Serial.print("target=");
        Serial.print(targetYaw);
        Serial.print(" yaw=");
        Serial.print(yaw);
        Serial.print(" turn=");
        Serial.print(turnCommand);
        Serial.print(" ball=");
        Serial.print(latestBallAngle);
        Serial.print(" hasBall=");
        Serial.println(hasBall);
    }
}
