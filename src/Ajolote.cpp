#include <Arduino.h>
#include <math.h>
#include "photo.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"
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
    RETREAT_TO_GOAL,
    DEFEND_GOAL,
    INTERCEPT_BALL
};

RobotState currentState = RobotState::RETREAT_TO_GOAL;

unsigned long retreatStartTime = 0;
unsigned long lastHomeGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
double targetYaw = 0.0;
uint16_t homeGoalSignature = 0;
float filteredDriveAngle = 0.0f;
float lastHomeGoalAngle = 0.0f;
float latestBallAngle = 0.0f;
bool avoidingLine = false;
int escapeAngle = 0;

const float homeDrivePwm = Constants::Goalie::kHomeDrivePwmRatio * Constants::Motor::maxPWM;
const float defenseDrivePwm = Constants::Goalie::kDefenseDrivePwmRatio * Constants::Motor::maxPWM;
const float interceptDrivePwm = Constants::Goalie::kInterceptDrivePwmRatio * Constants::Motor::maxPWM;
const float retreatDrivePwm = Constants::Goalie::kRetreatDrivePwmRatio * Constants::Motor::maxPWM;
const float goalSearchDrivePwm = Constants::Goalie::kGoalSearchDrivePwmRatio * Constants::Motor::maxPWM;

PID headingPD(
    Constants::Goalie::kHeadingKp,
    0.0f,
    Constants::Goalie::kHeadingKd,
    Constants::Goalie::kMaxTurnPwm,
    Constants::Goalie::kMinTurnPwm,
    Constants::Goalie::kHeadingSettleBandDeg
);

}

bool ballIsThreat(float ballAngle) {
    // The current UART sender only reports angle, so freshness plus angle window
    // decides whether the goalie should slide with the ball.
    return fabsf(ballAngle) <= Constants::Goalie::kIRThreatAngleToleranceDeg;
}

float makeInterceptAngle(float goalAngle, float ballAngle) {
    const float clampedBallAngle = DriveHelpers::clampSymmetric(
        ballAngle,
        Constants::Goalie::kBallAngleClampDeg);
    float driveAngle = Constants::Goalie::kBallFollowWeight *
                       (clampedBallAngle + (goalAngle * Constants::Goalie::kGoalCorrectionWeight));
    driveAngle = DriveHelpers::clampSymmetric(driveAngle, Constants::Goalie::kBallAngleClampDeg);

    if (fabsf(driveAngle) <= Constants::Goalie::kInterceptDeadbandDeg) {
        return 0.0f;
    }

    return driveAngle;
}

void recoverHomePosition(const PixyBlock& homeGoal, double turnCommand) {
    if (homeGoal.found) {
        const float goalAngle = pixyGetGoalDriveAngle(homeGoal,
                                                      Constants::Goalie::kGoalAngleClampDeg,
                                                      Constants::Goalie::kGoalTrackDirectionSign,
                                                      Constants::Goalie::kGoalCenterDeadbandDeg);
        lastHomeGoalAngle = goalAngle;
        filteredDriveAngle = DriveHelpers::smoothAngleChange(
            filteredDriveAngle,
            goalAngle,
            Constants::Striker::kAngleSmoothingAlpha);
        robot.motors.move(filteredDriveAngle, retreatDrivePwm, turnCommand);
        return;
    }

    filteredDriveAngle = 0.0f;

    if (lastHomeGoalAngle > 0.0f) {
        robot.motors.move(90.0f, goalSearchDrivePwm, turnCommand);
        return;
    }

    if (lastHomeGoalAngle < 0.0f) {
        robot.motors.move(270.0f, goalSearchDrivePwm, turnCommand);
        return;
    }

    robot.motors.stop();
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

        latestBallAngle = DriveHelpers::wrapAngle180(180.0f - incomingData.toFloat());
        lastBallReadMs = millis();
    }
}

void setup() {
    robot.begin();
    robot.motors.stop();
    delay(2000);

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    HWSERIAL.begin(9600);
    HWSERIAL.setTimeout(5);

    pixyInit();
    pixyLockGoalSignature(homeGoalSignature,
                          PixySig::kYellowGoal,
                          PixySig::kBlueGoal,
                          Constants::Goalie::kGoalCaptureTimeoutMs,
                          &lastHomeGoalSeenTime);

    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();
    retreatStartTime = millis();
}

void loop() {
    if (avoidingLine) {
        if (millis() - avoidStartTime >= Constants::kAvoidDurationMs) {
            robot.motors.stop();
            avoidingLine = false;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), defenseDrivePwm);
        }
        return;
    }

    const int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
    if (escapeAngleDetected != -1) {
        escapeAngle = escapeAngleDetected;
        avoidStartTime = millis();
        avoidingLine = true;
        robot.motors.move(static_cast<float>(escapeAngle), defenseDrivePwm);
        return;
    }

    const unsigned long nowMs = millis();
    const double currentYaw = robot.bno.GetBNOData();
    const double turnCommand = headingPD.calculate(targetYaw, currentYaw, true);

    updateBallAngle();

    if (homeGoalSignature == 0) {
        pixyLockGoalSignature(homeGoalSignature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              0,
                              &lastHomeGoalSeenTime);
    }

    const PixyBlock homeGoal = pixyReadLockedGoal(homeGoalSignature);
    if (homeGoal.found) {
        lastHomeGoalSeenTime = nowMs;
        lastHomeGoalAngle = pixyGetGoalDriveAngle(homeGoal,
                                                  Constants::Goalie::kGoalAngleClampDeg,
                                                  Constants::Goalie::kGoalTrackDirectionSign,
                                                  Constants::Goalie::kGoalCenterDeadbandDeg);
    }

    if (currentState == RobotState::RETREAT_TO_GOAL) {
        const bool retreatTimeElapsed =
            nowMs - retreatStartTime >= Constants::Goalie::kRetreatDurationMs;

        if (!retreatTimeElapsed || !homeGoal.found) {
            recoverHomePosition(homeGoal, turnCommand);
            return;
        }

        currentState = RobotState::DEFEND_GOAL;
        filteredDriveAngle = 0.0f;
    }

    const bool lostHomeGoal =
        homeGoalSignature == 0 ||
        (!homeGoal.found && nowMs - lastHomeGoalSeenTime >= Constants::Goalie::kGoalLostTimeoutMs);

    if (lostHomeGoal) {
        currentState = RobotState::RETREAT_TO_GOAL;
        retreatStartTime = nowMs;
        recoverHomePosition(homeGoal, turnCommand);
        return;
    }

    const float goalAngle = pixyGetGoalDriveAngle(homeGoal,
                                                  Constants::Goalie::kGoalAngleClampDeg,
                                                  Constants::Goalie::kGoalTrackDirectionSign,
                                                  Constants::Goalie::kGoalCenterDeadbandDeg);
    const bool goalOutsideLane = !pixyIsGoalInsideLane(homeGoal,
                                                       Constants::kLeftGoalKeeperTresholdX,
                                                       Constants::kRightGoalKeeperTresholdX);
    const bool goalNeedsCentering = !pixyIsGoalCentered(homeGoal, Constants::kGoalKeeperTresholdX);
    const bool goalTooFar = !pixyIsGoalYInRange(homeGoal,
                                                Constants::kMinGoalKeeperTresholdY,
                                                Constants::kMaxGoalKeeperTresholdY);

    if (goalOutsideLane || goalNeedsCentering) {
        currentState = RobotState::DEFEND_GOAL;
        filteredDriveAngle = DriveHelpers::smoothAngleChange(
            filteredDriveAngle,
            goalAngle,
            Constants::Striker::kAngleSmoothingAlpha);
        robot.motors.move(filteredDriveAngle, homeDrivePwm, turnCommand);
        return;
    }

    if (nowMs - lastBallReadMs <= Constants::kIRFreshDataTimeoutMs) {
        if (ballIsThreat(latestBallAngle) && !goalTooFar) {
            currentState = RobotState::INTERCEPT_BALL;

            // Blend goal correction into the ball slide so the goalie does not drift
            // out of its lane while tracking the incoming shot.
            const float interceptAngle = makeInterceptAngle(goalAngle, latestBallAngle);
            filteredDriveAngle = DriveHelpers::smoothAngleChange(
                filteredDriveAngle,
                interceptAngle,
                Constants::Striker::kAngleSmoothingAlpha);
            robot.motors.move(filteredDriveAngle, interceptDrivePwm, turnCommand);
            return;
        }
    }

    if (currentState == RobotState::INTERCEPT_BALL && goalTooFar) {
        currentState = RobotState::RETREAT_TO_GOAL;
        retreatStartTime = nowMs;
        recoverHomePosition(homeGoal, turnCommand);
        return;
    }

    currentState = RobotState::DEFEND_GOAL;

    if (goalAngle != 0.0f) {
        filteredDriveAngle = DriveHelpers::smoothAngleChange(
            filteredDriveAngle,
            goalAngle,
            Constants::Striker::kAngleSmoothingAlpha);
        robot.motors.move(filteredDriveAngle, defenseDrivePwm, turnCommand);
        return;
    }

    filteredDriveAngle = 0.0f;
    robot.motors.move(0.0f, 0.0f, turnCommand);
}
