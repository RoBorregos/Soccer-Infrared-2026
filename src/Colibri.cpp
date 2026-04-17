#include <Arduino.h>
#include <math.h>
#include "constants.h"
#include "PID.h"
#include "photo.h"
#include "robot.h"
#include "pixyLib.h"
#include "DriveHelpers.h"

#define HWSERIAL Serial3

namespace {

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kPhotoLeftSignalPin, Constants::kPhotoLeftMuxS0Pin, Constants::kPhotoLeftMuxS1Pin, Constants::kPhotoLeftMuxS2Pin,
    Constants::kPhotoRightSignalPin, Constants::kPhotoRightMuxS0Pin, Constants::kPhotoRightMuxS1Pin, Constants::kPhotoRightMuxS2Pin,
    Constants::kPhotoFrontSignalPin, Constants::kPhotoFrontMuxS0Pin, Constants::kPhotoFrontMuxS1Pin, Constants::kPhotoFrontMuxS2Pin
);

enum class RobotState {
    GET_BEHIND_BALL,
    CARRY_BALL_FORWARD,
    AVOIDING_LINE
};

enum class AvoidSide {
    NONE,
    FRONT,
    LEFT,
    RIGHT
};

struct LineState {
    bool left;
    bool right;
    bool front;
    AvoidSide detectedSide;
    int escapeAngle;
};

RobotState currentState = RobotState::GET_BEHIND_BALL;

const float kChaseDrivePwm = 0.46f * Constants::Motor::maxPWM;
const float kBehindBallDrivePwm = 0.40f * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = Constants::Striker::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float kCarryDrivePwm = 0.50f * Constants::Motor::maxPWM;
constexpr unsigned long kDebugPrintIntervalMs = 120;
constexpr bool kEnableLineAvoidance = true;
// Match PIDStickIntoOneDirection2 exactly for heading hold while Colibri chases.
constexpr float kHeadingKp = 1.5f;
constexpr float kHeadingKd = 0.10f;
constexpr float kMaxTurnPwm = 55.0f;
constexpr float kMinTurnPwm = 12.0f;
constexpr float kHeadingSettleBandDeg = 6.0f;
constexpr unsigned long kPIDLookForBallTimeoutMs = 250;
constexpr unsigned long kBallFrontConfirmMs = 180;
constexpr float kColibriBehindBallApproachGain = 0.75f;
constexpr float kColibriBehindBallApproachClampDeg = 65.0f;
constexpr float kColibriBehindBallOrbitDeg = 110.0f;

PID orbitHeadingPD(
    kHeadingKp,
    0.0f,
    kHeadingKd,
    kMaxTurnPwm,
    kMinTurnPwm,
    kHeadingSettleBandDeg
);

PID avoidHeadingPD(
    Constants::Striker::kAvoidHeadingKp,
    0.0f,
    Constants::Striker::kAvoidHeadingKd,
    Constants::Striker::kAvoidMinTurnPwm,
    Constants::Striker::kAvoidMaxTurnPwm,
    Constants::Striker::kAvoidHeadingSettleBandDeg
);

double startupYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long ballFrontStartMs = 0;
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastDebugHeaderMs = 0;
uint16_t opponentGoalSignature = 0;
bool pixyAvailable = false;
float latestBallAngle = 0.0f;
float latestRawBallTheta = 0.0f;
int escapeAngle = 0;
AvoidSide activeAvoidSide = AvoidSide::NONE;

}

bool hasFreshGoalTrack(unsigned long nowMs) {
    return opponentGoalSignature != 0 &&
           (nowMs - lastGoalSeenTime) <= Constants::Striker::kGoalLostTimeoutMs;
}

bool isBallInFrontFromRawTheta(float rawTheta) {
    const float frontError = DriveHelpers::wrapAngle180(180.0f - rawTheta);
    return fabsf(frontError) <= Constants::Striker::kBallFrontToleranceDeg;
}

float getBehindBallChaseAngle(float ballAngle) {
    const float absAngle = fabsf(ballAngle);

    if (absAngle <= Constants::Striker::kBallFrontToleranceDeg) {
        return ballAngle;
    }

    if (absAngle > 90.0f) {
        const float orbitDirection = (ballAngle >= 0.0f) ? 1.0f : -1.0f;
        return orbitDirection * kColibriBehindBallOrbitDeg;
    }

    const float approachOffset = DriveHelpers::clampSymmetric(
        ballAngle * kColibriBehindBallApproachGain,
        kColibriBehindBallApproachClampDeg);
    return DriveHelpers::wrapAngle180(ballAngle + approachOffset);
}

float getGoalCarryDriveAngle(const PixyBlock& goalBlock) {
    return pixyGetGoalDriveAngle(goalBlock,
                                 Constants::Striker::kGoalAngleClampDeg,
                                 1.0f,
                                 Constants::Striker::kGoalHeadingDeadbandDeg);
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

const char* boolText(bool value) {
    return value ? "true" : "false";
}

const char* avoidSideName(AvoidSide side) {
    switch (side) {
        case AvoidSide::FRONT:
            return "front";
        case AvoidSide::LEFT:
            return "left";
        case AvoidSide::RIGHT:
            return "right";
        case AvoidSide::NONE:
        default:
            return "none";
    }
}

LineState readLineState() {
    const bool front = phototransistor_sensors.HasLineOnSide(Side::Front);
    const bool left = phototransistor_sensors.HasLineOnSide(Side::Left);
    const bool right = phototransistor_sensors.HasLineOnSide(Side::Right);

    if (front) {
        return {left, right, front, AvoidSide::FRONT, 180};
    }

    if (left) {
        return {left, right, front, AvoidSide::LEFT, 90};
    }

    if (right) {
        return {left, right, front, AvoidSide::RIGHT, -90};
    }

    return {left, right, front, AvoidSide::NONE, -1};
}

void printDebugStatus(unsigned long nowMs,
                      double currentYaw,
                      float ballAngle,
                      float chaseAngle,
                      bool goalDetected,
                      double goalAngle,
                      uint32_t goalSize,
                      AvoidSide avoidSide,
                      bool ballInFront) {
    if (nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
        return;
    }

    if (lastDebugHeaderMs == 0 || nowMs - lastDebugHeaderMs >= 2000) {
        Serial.println("yaw\tball\tchase\tline\tgoal\tgoal_ang\tgoal_sz\tline_side\tfront");
        lastDebugHeaderMs = nowMs;
    }

    lastDebugPrintMs = nowMs;
    Serial.print(currentYaw, 1);
    Serial.print('\t');
    Serial.print(ballAngle, 1);
    Serial.print('\t');
    Serial.print(chaseAngle, 1);
    Serial.print('\t');
    Serial.print(boolText(avoidSide != AvoidSide::NONE));
    Serial.print('\t');
    Serial.print(boolText(goalDetected));
    Serial.print('\t');
    Serial.print(goalAngle, 1);
    Serial.print('\t');
    Serial.print(goalSize);
    Serial.print('\t');
    Serial.print(avoidSideName(avoidSide));
    Serial.print('\t');
    Serial.println(boolText(ballInFront));
}

void updateBallAngle() {
    if (HWSERIAL.available() > 0) {
        String incomingData = HWSERIAL.readStringUntil('\n');
        incomingData.trim();

        if (incomingData.length() > 0) {
            if (incomingData.startsWith("a ")) {
                // Match PIDLookForBall exactly: mirror the UART theta and keep it
                // normalized in the full signed chase range.
                const float incomingAngle = incomingData.substring(2).toFloat();
                latestRawBallTheta = incomingAngle;
                latestBallAngle = wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            } else if (!incomingData.startsWith("r ")) {
                const float incomingAngle = incomingData.toFloat();
                latestRawBallTheta = incomingAngle;
                latestBallAngle = wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Pixy2 - Detection Test");
    Serial.println("====================================");
    Serial.println("Starting I2C bus for Pixy...");

    const int8_t pixyStatus = pixy.init();
    if (pixyStatus == 0) {
        pixyAvailable = true;
        Serial.println("[SUCCESS] Pixy2 connected!");
    } else {
        pixyAvailable = false;
        Serial.print("ERROR: Pixy not found. Error code: ");
        Serial.println(pixyStatus);
        Serial.println("Check: 1. PCB traces, 2. swapped SCL/SDA wires, 3. PixyMon I2C mode");
        Serial.println("Continuing without Pixy goal tracking.");
    }

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    HWSERIAL.begin(9600);

    if (pixyAvailable) {
        pixyLockGoalSignature(opponentGoalSignature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              Constants::Striker::kGoalCaptureTimeoutMs,
                              &lastGoalSeenTime);
    }

    startupYaw = robot.bno.GetBNOData();
    orbitHeadingPD.reset();
    avoidHeadingPD.reset();
    motorsEnableMs = millis() + Constants::Striker::kStartupHoldMs;
    Serial.println("Colibri debug ready");
}

void loop() {
    const unsigned long nowMs = millis();

    if (nowMs < motorsEnableMs) {
        robot.motors.stop();
        return;
    }

    updateBallAngle();
    const bool hasBall = (millis() - lastBallReadMs) <= kPIDLookForBallTimeoutMs;
    // Keep the chase-angle handoff identical to PIDLookForBall: use the latest
    // mirrored ball angle directly whenever the UART data is still fresh.
    const float chaseAngle = hasBall ? latestBallAngle : 0.0f;

    const double currentYaw = robot.bno.GetBNOData();

    PixyBlock opponentGoal = pixyNoGoalBlock();
    if (pixyAvailable) {
        if (opponentGoalSignature == 0) {
            pixyLockGoalSignature(opponentGoalSignature,
                                  PixySig::kYellowGoal,
                                  PixySig::kBlueGoal,
                                  0,
                                  &lastGoalSeenTime);
        }

        opponentGoal = pixyReadLockedGoal(opponentGoalSignature);
        if (opponentGoal.found) {
            lastGoalSeenTime = nowMs;
        }
    }

    const double avoidTurnCommand = avoidHeadingPD.calculate(startupYaw, currentYaw, true);

    if (kEnableLineAvoidance && currentState == RobotState::AVOIDING_LINE) {
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         opponentGoal.found,
                         opponentGoal.angle,
                         opponentGoal.area,
                         activeAvoidSide,
                         isBallInFrontFromRawTheta(latestRawBallTheta));
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::GET_BEHIND_BALL;
            activeAvoidSide = AvoidSide::NONE;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        }
        return;
    }

    if (kEnableLineAvoidance) {
        const LineState lineState = readLineState();
        if (lineState.escapeAngle != -1) {
            escapeAngle = lineState.escapeAngle;
            avoidStartTime = nowMs;
            currentState = RobotState::AVOIDING_LINE;
            activeAvoidSide = lineState.detectedSide;
            printDebugStatus(nowMs,
                             currentYaw,
                             latestBallAngle,
                             chaseAngle,
                             opponentGoal.found,
                             opponentGoal.angle,
                             opponentGoal.area,
                             activeAvoidSide,
                             isBallInFrontFromRawTheta(latestRawBallTheta));
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
            return;
        }
    }

    activeAvoidSide = AvoidSide::NONE;

    if (!hasBall) {
        const double orbitTurnCommand = orbitHeadingPD.calculate(startupYaw, currentYaw, true);
        printDebugStatus(nowMs, currentYaw, latestBallAngle, chaseAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, false);
        robot.motors.move(0.0f, 0.0f, orbitTurnCommand);
        return;
    }

    const float ballAngle = chaseAngle;
    const float chaseDriveAngle = getBehindBallChaseAngle(ballAngle);
    const bool ballInFront = isBallInFrontFromRawTheta(latestRawBallTheta);

    if (ballInFront) {
        if (ballFrontStartMs == 0) {
            ballFrontStartMs = nowMs;
        }
    } else {
        ballFrontStartMs = 0;
    }

    const bool hasConfirmedFrontBall =
        ballFrontStartMs != 0 && (nowMs - ballFrontStartMs) >= kBallFrontConfirmMs;
    const bool canCarryToGoal =
        pixyAvailable &&
        hasFreshGoalTrack(nowMs) &&
        opponentGoal.found &&
        hasConfirmedFrontBall;
    const double orbitTurnCommand = orbitHeadingPD.calculate(startupYaw, currentYaw, true);
    const float carryDriveAngle = getGoalCarryDriveAngle(opponentGoal);
    const float chaseDrivePwm = (fabsf(ballAngle) > 90.0f) ? kBehindBallDrivePwm : kChaseDrivePwm;

    if (canCarryToGoal) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        // Once the ball is in front, Pixy only nudges the translation angle
        // toward the goal instead of taking over the robot heading PID.
        printDebugStatus(nowMs, currentYaw, ballAngle, carryDriveAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, ballInFront);
        robot.motors.move(carryDriveAngle, kCarryDrivePwm, orbitTurnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    // Bias the chase angle around the ball so Colibri gets behind it instead of
    // just driving straight through the rear side.
    printDebugStatus(nowMs, currentYaw, ballAngle, chaseDriveAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, ballInFront);
    robot.motors.move(chaseDriveAngle, chaseDrivePwm, orbitTurnCommand);
}
