#include <Arduino.h>
#include "constants.h"
#include "DriveHelpers.h"
#include "PID.h"
#include "pixyLib.h"
#include "robot.h"

#define HWSERIAL Serial3

namespace {

Robot robot;

constexpr float kHeadingKp = 1.5f;
constexpr float kHeadingKd = 0.10f;
constexpr float kMaxTurnPwm = 65.0f;
constexpr float kMinTurnPwm = 40.0f;
constexpr float kHeadingSettleBandDeg = 5.5f;
constexpr unsigned long kDebugIntervalMs = 120;
constexpr unsigned long kBallTimeoutMs = 250;
constexpr float kBallFrontMinAngleDeg = 150.0f;
constexpr float kBallFrontMaxAngleDeg = 180.0f;
constexpr float kGoalAimEnterArea = 1800.0f;
constexpr float kGoalAimExitArea = 1500.0f;
constexpr float kGoalAreaFilterAlpha = 0.28f;
constexpr float kGoalAngleDeadbandDeg = 2.5f;

PID headingPD(kHeadingKp, 0.0f, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);

double startupYaw = 0.0;
float latestBallAngle = 0.0f;
float filteredGoalArea = 0.0f;
unsigned long lastBallReadMs = 0;
unsigned long lastDebugMs = 0;
uint16_t lockedGoalSignature = 0;
bool goalCloseEnoughLatched = false;

}

void updateBallAngle() {
    if (HWSERIAL.available() <= 0) {
        return;
    }

    String incomingData = HWSERIAL.readStringUntil('\n');
    incomingData.trim();

    if (incomingData.length() == 0) {
        return;
    }

    latestBallAngle = incomingData.toFloat();
    lastBallReadMs = millis();
}

bool hasFreshBall(unsigned long nowMs) {
    return lastBallReadMs != 0 && (nowMs - lastBallReadMs) <= kBallTimeoutMs;
}

bool isBallInFrontWindow(float rawBallAngle) {
    const float absBallAngle = fabsf(rawBallAngle);
    return absBallAngle >= kBallFrontMinAngleDeg &&
           absBallAngle <= kBallFrontMaxAngleDeg;
}

double getGoalAimTargetYaw(const PixyBlock& goal, double currentYaw) {
    if (!goal.found || fabs(goal.angle) <= kGoalAngleDeadbandDeg) {
        return currentYaw;
    }

    return DriveHelpers::wrapAngle180(currentYaw + goal.angle);
}

PixyBlock readBestGoal() {
    if (lockedGoalSignature == 0) {
        lockedGoalSignature = pixyChooseGoalSignature(PixySig::kYellowGoal, PixySig::kBlueGoal);
    }

    PixyBlock goal = pixyReadLockedGoal(lockedGoalSignature);
    if (goal.found) {
        return goal;
    }

    lockedGoalSignature = pixyChooseGoalSignature(PixySig::kYellowGoal, PixySig::kBlueGoal);
    return pixyReadLockedGoal(lockedGoalSignature);
}

bool updateGoalCloseEnough(const PixyBlock& goal) {
    if (!goal.found) {
        filteredGoalArea = 0.0f;
        goalCloseEnoughLatched = false;
        return false;
    }

    if (filteredGoalArea <= 0.0f) {
        filteredGoalArea = static_cast<float>(goal.area);
    } else {
        filteredGoalArea += (static_cast<float>(goal.area) - filteredGoalArea) * kGoalAreaFilterAlpha;
    }

    if (!goalCloseEnoughLatched && filteredGoalArea >= kGoalAimEnterArea) {
        goalCloseEnoughLatched = true;
    } else if (goalCloseEnoughLatched && filteredGoalArea <= kGoalAimExitArea) {
        goalCloseEnoughLatched = false;
    }

    return goalCloseEnoughLatched;
}

void printDebug(unsigned long nowMs,
                const char* mode,
                double yaw,
                double targetYaw,
                float turnCommand,
                float ballAngle,
                float ballAbsAngle,
                bool hasBall,
                bool ballInFrontWindow,
                const PixyBlock& goal,
                uint16_t goalSignature,
                bool goalCloseEnough) {
    if (nowMs - lastDebugMs < kDebugIntervalMs) {
        return;
    }

    lastDebugMs = nowMs;
    Serial.print("mode=");
    Serial.print(mode);
    Serial.print(" yaw=");
    Serial.print(yaw, 1);
    Serial.print(" target=");
    Serial.print(targetYaw, 1);
    Serial.print(" turn=");
    Serial.print(turnCommand, 1);
    Serial.print(" ball=");
    Serial.print(ballAngle, 1);
    Serial.print(" ball_abs=");
    Serial.print(ballAbsAngle, 1);
    Serial.print(" has_ball=");
    Serial.print(hasBall ? "true" : "false");
    Serial.print(" ball_front=");
    Serial.print(ballInFrontWindow ? "true" : "false");
    Serial.print(" goal_sig=");
    Serial.print(goalSignature);
    Serial.print(" goal_found=");
    Serial.print(goal.found ? "true" : "false");
    Serial.print(" goal_close=");
    Serial.print(goalCloseEnough ? "true" : "false");
    Serial.print(" goal_ang=");
    Serial.print(goal.angle, 1);
    Serial.print(" goal_area_raw=");
    Serial.print(goal.area);
    Serial.print(" goal_area_filt=");
    Serial.println(filteredGoalArea, 1);
}

void setup() {
    Serial.begin(115200);
    HWSERIAL.begin(Constants::kIRSerialBaud);
    HWSERIAL.setTimeout(5);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    startupYaw = robot.bno.GetBNOData();
    headingPD.reset();

    const int pixyStatus = pixy.init();
    if (pixyStatus == 0) {
        Serial.println("[SUCCESS] Pixy2 connected");
    } else {
        Serial.print("[FAIL] Pixy2 init failed: ");
        Serial.println(pixyStatus);
        while (true) {
            delay(100);
        }
    }

    Serial.println("Goal aim heading calibration ready");
    Serial.print("ball_front_min_deg=");
    Serial.println(kBallFrontMinAngleDeg, 1);
    Serial.print("ball_front_max_deg=");
    Serial.println(kBallFrontMaxAngleDeg, 1);
    Serial.print("goal_area_enter=");
    Serial.println(kGoalAimEnterArea, 1);
    Serial.print("goal_area_exit=");
    Serial.println(kGoalAimExitArea, 1);
}

void loop() {
    const unsigned long nowMs = millis();
    updateBallAngle();

    const double yaw = robot.bno.GetBNOData();
    const bool hasBall = hasFreshBall(nowMs);
    const float ballAbsAngle = fabsf(latestBallAngle);
    const bool ballInFrontWindow = hasBall && isBallInFrontWindow(latestBallAngle);

    const PixyBlock goal = readBestGoal();
    const uint16_t goalSignature = lockedGoalSignature;
    const bool goalCloseEnough = updateGoalCloseEnough(goal);

    const bool shouldAimGoal = ballInFrontWindow && goalCloseEnough;
    const double targetYaw = shouldAimGoal
        ? getGoalAimTargetYaw(goal, yaw)
        : startupYaw;
    const float turnCommand = static_cast<float>(headingPD.calculate(targetYaw, yaw, true));

    printDebug(nowMs,
               shouldAimGoal ? "goal_aim" : "hold_startup",
               yaw,
               targetYaw,
               turnCommand,
               latestBallAngle,
               ballAbsAngle,
               hasBall,
               ballInFrontWindow,
               goal,
               goalSignature,
               goalCloseEnough);

    robot.motors.move(0.0f, 0.0f, turnCommand);
}
