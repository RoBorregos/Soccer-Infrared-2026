#include <Arduino.h>
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;

double targetYaw = 0.0;

const float kHeadingKp = 1.5f;
const float kHeadingKd = 0.10f;
const float kMaxTurnPwm = 55.0f;
const float kMinTurnPwm = 12.0f;
const float kHeadingSettleBandDeg = 6.0f;
const unsigned long kDebugIntervalMs = 100;

PID headingPD(kHeadingKp, 0.0f, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);
const float drivePwm = 0.50f * Constants::Motor::maxPWM;


void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.motors.stop();
    delay(2000);

    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();
}

void loop() {
    static unsigned long lastDebugMs = 0;

    const double yaw = robot.bno.GetBNOData();
    const double turnCommand = headingPD.calculate(targetYaw, yaw, true);
    robot.motors.move(0.0f, drivePwm, turnCommand);

    if (millis() - lastDebugMs >= kDebugIntervalMs) {
        lastDebugMs = millis();
        Serial.print("target=");
        Serial.print(targetYaw);
        Serial.print(" yaw=");
        Serial.print(yaw);
        Serial.print(" turn=");
        Serial.println(turnCommand);
    }
}
