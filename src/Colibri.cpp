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

RobotState currentState = RobotState::GET_BEHIND_BALL;

const float kCarryDrivePwm = Constants::Striker::kGoalDrivePwmRatio * Constants::Motor::maxPWM;
const float kBehindBallDrivePwm = Constants::Striker::kChaseDrivePwmRatio * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = Constants::Striker::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float kBallFrontToleranceDeg = 5.0f;
const unsigned long kStartupHoldMs = 1500;

PID headingPD(
    Constants::Striker::kHeadingKp,
    0.0f,
    Constants::Striker::kHeadingKd,
    Constants::Striker::kMaxTurnPwm,
    Constants::Striker::kMinTurnPwm,
    Constants::Striker::kHeadingSettleBandDeg
);

double startupYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
uint16_t opponentGoalSignature = 0;
bool pixyAvailable = false;
float latestBallAngle = 0.0f;
int escapeAngle = 0;

}

bool hasFreshGoalTrack(unsigned long nowMs) {
    return opponentGoalSignature != 0 &&
           (nowMs - lastGoalSeenTime) <= Constants::Striker::kGoalLostTimeoutMs;
}

bool isBallInFront(float ballAngle) {
    return fabsf(ballAngle) <= kBallFrontToleranceDeg;
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
    headingPD.reset();
    motorsEnableMs = millis() + kStartupHoldMs;
}

void loop() {
    const unsigned long nowMs = millis();

    if (nowMs < motorsEnableMs) {
        robot.motors.stop();
        return;
    }

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

    const double currentYaw = robot.bno.GetBNOData();
    const bool goalCloseEnough = pixyAvailable &&
                                 pixyIsGoalCloseEnough(opponentGoal, Constants::Striker::kGoalAimAreaThreshold);
    double headingTarget = startupYaw;

    if (goalCloseEnough) {
        headingTarget = pixyGetHeadingTargetForGoal(opponentGoal,
                                                    currentYaw,
                                                    Constants::Striker::kGoalAimAreaThreshold,
                                                    Constants::Striker::kGoalHeadingDeadbandDeg);
    }

    const double turnCommand = headingPD.calculate(headingTarget, currentYaw, true);

    if (currentState == RobotState::AVOIDING_LINE) {
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::GET_BEHIND_BALL;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, turnCommand);
        }
        return;
    }

    const int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
    if (escapeAngleDetected != -1) {
        escapeAngle = escapeAngleDetected;
        avoidStartTime = nowMs;
        currentState = RobotState::AVOIDING_LINE;
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, turnCommand);
        return;
    }

    updateBallAngle();

    if (nowMs - lastBallReadMs > Constants::kIRFreshDataTimeoutMs) {
        robot.motors.move(0.0f, 0.0f, turnCommand);
        return;
    }

    const float ballAngle = latestBallAngle;

    if (isBallInFront(ballAngle)) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        robot.motors.move(0.0f, kCarryDrivePwm, turnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    robot.motors.move(ballAngle, kBehindBallDrivePwm, turnCommand);
}
