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
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
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
const unsigned long kStartupHoldMs = 1500;

PID headingPD(
    Constants::Striker::kHeadingKp,
    0.0f,
    Constants::Striker::kHeadingKd,
    Constants::Striker::kMaxTurnPwm,
    Constants::Striker::kMinTurnPwm,
    Constants::Striker::kHeadingSettleBandDeg
);

double targetYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
uint16_t opponentGoalSignature = 0;
float latestBallAngle = 0.0f;
int escapeAngle = 0;

}

bool hasFreshGoalTrack(unsigned long nowMs) {
    return opponentGoalSignature != 0 &&
           (nowMs - lastGoalSeenTime) <= Constants::Striker::kGoalLostTimeoutMs;
}

bool isBallInFront(float ballAngle) {
    return fabsf(ballAngle) <= Constants::Striker::kBallFrontToleranceDeg;
}

float makeBehindBallDriveAngle(float ballAngle) {
    const float absBallAngle = fabsf(ballAngle);

    if (absBallAngle <= Constants::Striker::kBallFrontToleranceDeg) {
        return 0.0f;
    }

    if (absBallAngle <= 90.0f) {
        const float approachAngle = ballAngle * Constants::Striker::kBehindBallApproachGain;
        return DriveHelpers::clampSymmetric(
            approachAngle,
            Constants::Striker::kBehindBallApproachClampDeg);
    }

    return (ballAngle > 0.0f)
        ? Constants::Striker::kBehindBallOrbitAngleDeg
        : -Constants::Striker::kBehindBallOrbitAngleDeg;
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

    pixyInit();
    pixyLockGoalSignature(opponentGoalSignature,
                          PixySig::kYellowGoal,
                          PixySig::kBlueGoal,
                          Constants::Striker::kGoalCaptureTimeoutMs,
                          &lastGoalSeenTime);

    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();
    motorsEnableMs = millis() + kStartupHoldMs;
}

void loop() {
    const unsigned long nowMs = millis();

    if (nowMs < motorsEnableMs) {
        robot.motors.stop();
        return;
    }

    if (currentState == RobotState::AVOIDING_LINE) {
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            robot.motors.stop();
            currentState = RobotState::GET_BEHIND_BALL;
        } else {
            // Keep line escape identical to the standalone avoidLine test behavior.
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm);
        }
        return;
    }

    const int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
    if (escapeAngleDetected != -1) {
        escapeAngle = escapeAngleDetected;
        avoidStartTime = nowMs;
        currentState = RobotState::AVOIDING_LINE;
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm);
        return;
    }

    updateBallAngle();

    if (opponentGoalSignature == 0) {
        pixyLockGoalSignature(opponentGoalSignature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              0,
                              &lastGoalSeenTime);
    }

    const PixyBlock opponentGoal = pixyReadLockedGoal(opponentGoalSignature);
    if (opponentGoal.found) {
        lastGoalSeenTime = nowMs;
    }

    const double currentYaw = robot.bno.GetBNOData();

    if (nowMs - lastBallReadMs > Constants::kIRFreshDataTimeoutMs) {
        const double turnCommand = headingPD.calculate(targetYaw, currentYaw, true);
        robot.motors.move(0.0f, 0.0f, turnCommand);
        return;
    }

    const float ballAngle = latestBallAngle;

    // Keep the front of the robot pointed toward the goal only when Pixy says the
    // goal is already close enough to matter. Otherwise keep the current heading stable.
    targetYaw = pixyGetHeadingTargetForGoal(opponentGoal,
                                            currentYaw,
                                            Constants::Striker::kGoalAimAreaThreshold,
                                            Constants::Striker::kGoalHeadingDeadbandDeg);
    const double turnCommand = headingPD.calculate(targetYaw, currentYaw, true);

    if (isBallInFront(ballAngle)) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        robot.motors.move(0.0f, kCarryDrivePwm, turnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    const float driveAngle = makeBehindBallDriveAngle(ballAngle);
    robot.motors.move(driveAngle, kBehindBallDrivePwm, turnCommand);
}
