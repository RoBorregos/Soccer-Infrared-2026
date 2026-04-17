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

const float kChaseDrivePwm = Constants::Striker::kChaseDrivePwmRatio * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = Constants::Striker::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float kCarryDrivePwm = Constants::Striker::kGoalDrivePwmRatio * Constants::Motor::maxPWM;
constexpr unsigned long kDebugPrintIntervalMs = 120;
constexpr float kPIDLookForBallHeadingKp = 2.5f;
constexpr float kPIDLookForBallHeadingKd = 0.10f;
constexpr float kPIDLookForBallMaxTurnPwm = 70.0f;
constexpr float kPIDLookForBallMinTurnPwm = 15.0f;
constexpr float kPIDLookForBallHeadingSettleBandDeg = 6.5f;
const float kPIDLookForBallDrivePwm = 0.60f * Constants::Motor::maxPWM;
constexpr unsigned long kPIDLookForBallTimeoutMs = 250;

PID orbitHeadingPD(
    kPIDLookForBallHeadingKp,
    0.0f,
    kPIDLookForBallHeadingKd,
    kPIDLookForBallMinTurnPwm,
    kPIDLookForBallMaxTurnPwm,
    kPIDLookForBallHeadingSettleBandDeg
);

PID carryHeadingPD(
    Constants::Striker::kHeadingKp,
    0.0f,
    Constants::Striker::kHeadingKd,
    Constants::Striker::kMinTurnPwm,
    Constants::Striker::kMaxTurnPwm,
    Constants::Striker::kHeadingSettleBandDeg
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
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastDebugHeaderMs = 0;
uint16_t opponentGoalSignature = 0;
bool pixyAvailable = false;
float latestBallAngle = 0.0f;
int escapeAngle = 0;
AvoidSide activeAvoidSide = AvoidSide::NONE;

}

bool hasFreshGoalTrack(unsigned long nowMs) {
    return opponentGoalSignature != 0 &&
           (nowMs - lastGoalSeenTime) <= Constants::Striker::kGoalLostTimeoutMs;
}

bool isBallInFront(float ballAngle) {
    return fabsf(ballAngle) <= Constants::Striker::kBallFrontToleranceDeg;
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
                // Match communicationWithTeensy: read the theta text exactly as it
                // arrives from Serial3 and feed that directly into the chase logic.
                const float incomingAngle = incomingData.substring(2).toFloat();
                latestBallAngle = incomingAngle;
                lastBallReadMs = millis();
            } else if (!incomingData.startsWith("r ")) {
                const float incomingAngle = incomingData.toFloat();
                latestBallAngle = incomingAngle;
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
        while (true) {
        }
    }

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    delay(1000);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    HWSERIAL.begin(9600);
    HWSERIAL.setTimeout(5);

    if (pixyAvailable) {
        pixyLockGoalSignature(opponentGoalSignature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              Constants::Striker::kGoalCaptureTimeoutMs,
                              &lastGoalSeenTime);
    }

    startupYaw = robot.bno.GetBNOData();
    orbitHeadingPD.reset();
    carryHeadingPD.reset();
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

    if (currentState == RobotState::AVOIDING_LINE) {
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         opponentGoal.found,
                         opponentGoal.angle,
                         opponentGoal.area,
                         activeAvoidSide,
                         isBallInFront(latestBallAngle));
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::GET_BEHIND_BALL;
            activeAvoidSide = AvoidSide::NONE;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        }
        return;
    }

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
                         isBallInFront(latestBallAngle));
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        return;
    }

    activeAvoidSide = AvoidSide::NONE;

    if (!hasBall) {
        const double orbitTurnCommand = orbitHeadingPD.calculate(startupYaw, currentYaw, true);
        printDebugStatus(nowMs, currentYaw, latestBallAngle, chaseAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, false);
        robot.motors.move(0.0f, 0.0f, orbitTurnCommand);
        return;
    }

    const float ballAngle = latestBallAngle;
    const bool goalCloseEnough = pixyAvailable &&
                                 pixyIsGoalCloseEnough(opponentGoal, Constants::Striker::kGoalAimAreaThreshold);
    const bool ballInFront = isBallInFront(ballAngle);
    const bool canCarryToGoal = goalCloseEnough && ballInFront;
    double headingTarget = startupYaw;

    if (canCarryToGoal) {
        headingTarget = pixyGetHeadingTargetForGoal(opponentGoal,
                                                    currentYaw,
                                                    Constants::Striker::kGoalAimAreaThreshold,
                                                    Constants::Striker::kGoalHeadingDeadbandDeg);
    }

    const double carryTurnCommand = carryHeadingPD.calculate(headingTarget, currentYaw, true);
    const double orbitTurnCommand = orbitHeadingPD.calculate(startupYaw, currentYaw, true);

    if (canCarryToGoal) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        printDebugStatus(nowMs, currentYaw, ballAngle, ballAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, ballInFront);
        robot.motors.move(0.0f, kCarryDrivePwm, carryTurnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    // Keep Colibri's normal chase orbit identical to the PIDLookForBall test.
    printDebugStatus(nowMs, currentYaw, ballAngle, ballAngle, opponentGoal.found, opponentGoal.angle, opponentGoal.area, activeAvoidSide, ballInFront);
    robot.motors.move(ballAngle, kPIDLookForBallDrivePwm, orbitTurnCommand);
}
