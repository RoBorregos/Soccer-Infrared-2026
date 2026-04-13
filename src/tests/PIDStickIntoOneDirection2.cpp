#include <Arduino.h>
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;

double targetYaw = 0.0;

const float kHeadingKp = 2.3f;
const float kHeadingKd = 0.18f;
const float kMaxTurnPwm = 90.0f;
const float kMinTurnPwm = 24.0f;
const float kHeadingSettleBandDeg = 5.5f;
const unsigned long kDebugIntervalMs = 100;

PID headingPD(kHeadingKp, 0.0f, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);
const float drivePwm = 0.40f * Constants::Motor::maxPWM;


void setup() {
    Serial.begin(115200);
    robot.begin();
    delay(2000);

    targetYaw = robot.imu.getAngle();
}

void loop() {
    double yaw = robot.imu.getAngle();
    double turnCommand = headingPD.calculate(targetYaw, yaw);
    robot.motors.move(0, drivePwm, turnCommand);
}