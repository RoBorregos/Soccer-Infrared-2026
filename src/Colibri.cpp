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

PID chaseHeadingPD(
    Constants::Striker::kHeadingKp,
    0.0f,
    Constants::Striker::kHeadingKd,
    Constants::Striker::kMaxTurnPwm,
    Constants::Striker::kMinTurnPwm,
    Constants::Striker::kHeadingSettleBandDeg
);

PID avoidHeadingPD(
    Constants::Striker::kAvoidHeadingKp,
    0.0f,
    Constants::Striker::kAvoidHeadingKd,
    Constants::Striker::kAvoidMaxTurnPwm,
    Constants::Striker::kAvoidMinTurnPwm,
    Constants::Striker::kAvoidHeadingSettleBandDeg
);

double startupYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
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
                      float ballAngle,
                      bool goalCloseEnough,
                      AvoidSide avoidSide,
                      bool ballInFront) {
    if (nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
        return;
    }

    lastDebugPrintMs = nowMs;
    Serial.print("ball_angle=");
    Serial.print(ballAngle, 1);
    Serial.print(" goal_close_enough=");
    Serial.print(boolText(goalCloseEnough));
    Serial.print(" avoiding_line=");
    Serial.print(boolText(avoidSide != AvoidSide::NONE));
    Serial.print(" line_side=");
    Serial.print(avoidSideName(avoidSide));
    Serial.print(" ball_in_front=");
    Serial.println(boolText(ballInFront));
}

void updateBallAngle() {
    while (HWSERIAL.available() > 0) {
        String incomingData = HWSERIAL.readStringUntil('\n');
        incomingData.trim();

        if (incomingData.length() == 0) {
            continue;
        }

        if (incomingData.startsWith("a ")) {
            incomingData = incomingData.substring(2);
        }

        // The Uno sender publishes the ball direction in its own reference frame.
        latestBallAngle = DriveHelpers::wrapAngle180(180.0f - incomingData.toFloat());
        lastBallReadMs = millis();
    }
}

void setup() {
    Serial.begin(115200);

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    delay(1000);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    HWSERIAL.begin(9600);
    HWSERIAL.setTimeout(5);

    // Colibri can still run as ball-follow + line-avoid without a Pixy attached.
    pixyAvailable = (pixy.init() == 0);
    if (pixyAvailable) {
        pixyLockGoalSignature(opponentGoalSignature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              Constants::Striker::kGoalCaptureTimeoutMs,
                              &lastGoalSeenTime);
    }

    startupYaw = robot.bno.GetBNOData();
    chaseHeadingPD.reset();
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
                         latestBallAngle,
                         false,
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
                         latestBallAngle,
                         false,
                         activeAvoidSide,
                         isBallInFront(latestBallAngle));
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        return;
    }

    activeAvoidSide = AvoidSide::NONE;

    updateBallAngle();

    if (nowMs - lastBallReadMs > Constants::kIRFreshDataTimeoutMs) {
        const double chaseTurnCommand = chaseHeadingPD.calculate(startupYaw, currentYaw, true);
        printDebugStatus(nowMs, latestBallAngle, false, activeAvoidSide, false);
        robot.motors.move(0.0f, 0.0f, chaseTurnCommand);
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

    const double chaseTurnCommand = chaseHeadingPD.calculate(headingTarget, currentYaw, true);
    printDebugStatus(nowMs, ballAngle, goalCloseEnough, activeAvoidSide, ballInFront);

    if (canCarryToGoal) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        robot.motors.move(0.0f, kCarryDrivePwm, chaseTurnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    robot.motors.move(ballAngle, kChaseDrivePwm, chaseTurnCommand);
}
