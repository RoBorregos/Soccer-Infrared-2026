#include <Arduino.h>
#include <math.h>
#include "constants.h"
#include "IMU.h"
#include "PID.h"
#include "photo.h"
#include "robot.h"
#include "pixyLib.h"
#include "DriveHelpers.h"

#define HWSERIAL Serial3

namespace {

Robot robot;
IMUDriver ajoloteImu(&IMU, IMU_ADDRESS);
Phototransistor phototransistor_sensors(
    Constants::kPhotoLeftSignalPin, Constants::kPhotoLeftMuxS0Pin, Constants::kPhotoLeftMuxS1Pin, Constants::kPhotoLeftMuxS2Pin,
    Constants::kPhotoRightSignalPin, Constants::kPhotoRightMuxS0Pin, Constants::kPhotoRightMuxS1Pin, Constants::kPhotoRightMuxS2Pin,
    Constants::kPhotoFrontSignalPin, Constants::kPhotoFrontMuxS0Pin, Constants::kPhotoFrontMuxS1Pin, Constants::kPhotoFrontMuxS2Pin
);

enum class RobotState {
    WAITING_FOR_HOME_GOAL,
    RECOVERING_HOME_GOAL,
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

RobotState currentState = RobotState::WAITING_FOR_HOME_GOAL;

const float kChaseDrivePwm = 0.46f * Constants::Motor::maxPWM;
const float kBehindBallDrivePwm = 0.40f * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = Constants::Ajolote::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float kCarryDrivePwm = Constants::Ajolote::kChaseDrivePwmRatio * Constants::Motor::maxPWM;
const float kHomeRecoveryDrivePwm = Constants::Ajolote::kHomeRecoveryDrivePwmRatio * Constants::Motor::maxPWM;
const float kGoalSearchDrivePwm = Constants::Ajolote::kGoalSearchDrivePwmRatio * Constants::Motor::maxPWM;
constexpr unsigned long kDebugPrintIntervalMs = 120;
constexpr float kHeadingKp = 1.5f;
constexpr float kHeadingKd = 0.10f;
constexpr float kMaxTurnPwm = 65.0f;
constexpr float kMinTurnPwm = 40.0f;
constexpr float kHeadingSettleBandDeg = 5.5f;
constexpr unsigned long kPIDLookForBallTimeoutMs = 250;
constexpr unsigned long kBallFrontConfirmMs = 180;
constexpr float kAjoloteBehindBallApproachGain = 0.75f;
constexpr float kAjoloteBehindBallApproachClampDeg = 65.0f;
constexpr float kAjoloteBehindBallOrbitDeg = 110.0f;

PID orbitHeadingPD(
    kHeadingKp,
    0.0f,
    kHeadingKd,
    kMaxTurnPwm,
    kMinTurnPwm,
    kHeadingSettleBandDeg
);

PID avoidHeadingPD(
    kHeadingKp,
    0.0f,
    kHeadingKd,
    kMaxTurnPwm,
    kMinTurnPwm,
    kHeadingSettleBandDeg
);

double startupYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastHomeGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long ballFrontStartMs = 0;
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastDebugHeaderMs = 0;
uint16_t homeGoalSignature = 0;
bool imuAvailable = false;
bool pixyAvailable = true;
float latestBallAngle = 0.0f;
float latestRawBallTheta = 0.0f;
float filteredRecoveryAngle = 0.0f;
float lastHomeGoalDriveAngle = 0.0f;
int escapeAngle = 0;
AvoidSide activeAvoidSide = AvoidSide::NONE;

}

bool hasFreshHomeGoalTrack(unsigned long nowMs) {
    return homeGoalSignature != 0 &&
           (nowMs - lastHomeGoalSeenTime) <= Constants::Ajolote::kGoalLostTimeoutMs;
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
        return orbitDirection * kAjoloteBehindBallOrbitDeg;
    }

    const float approachOffset = DriveHelpers::clampSymmetric(
        ballAngle * kAjoloteBehindBallApproachGain,
        kAjoloteBehindBallApproachClampDeg);
    return DriveHelpers::wrapAngle180(ballAngle + approachOffset);
}

float getHomeGoalDriveAngle(const PixyBlock& homeGoal, float deadbandDeg) {
    return pixyGetGoalDriveAngle(homeGoal,
                                 Constants::Ajolote::kHomeGoalAngleClampDeg,
                                 Constants::Goalie::kGoalTrackDirectionSign,
                                 deadbandDeg);
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

const char* stateName(RobotState state) {
    switch (state) {
        case RobotState::WAITING_FOR_HOME_GOAL:
            return "waiting_goal";
        case RobotState::RECOVERING_HOME_GOAL:
            return "recover_goal";
        case RobotState::GET_BEHIND_BALL:
            return "get_behind";
        case RobotState::CARRY_BALL_FORWARD:
            return "carry_ball";
        case RobotState::AVOIDING_LINE:
            return "avoid_line";
        default:
            return "unknown";
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
                      const PixyBlock& homeGoal,
                      bool homeGoalCloseEnough,
                      AvoidSide avoidSide,
                      bool ballInFront) {
    if (nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
        return;
    }

    if (lastDebugHeaderMs == 0 || nowMs - lastDebugHeaderMs >= 2000) {
        Serial.println("state\tyaw\tball\tchase\tgoal\tgoal_big\tgoal_ang\tgoal_sz\tline_side\tfront");
        lastDebugHeaderMs = nowMs;
    }

    lastDebugPrintMs = nowMs;
    Serial.print(stateName(currentState));
    Serial.print('\t');
    Serial.print(currentYaw, 1);
    Serial.print('\t');
    Serial.print(ballAngle, 1);
    Serial.print('\t');
    Serial.print(chaseAngle, 1);
    Serial.print('\t');
    Serial.print(boolText(hasFreshHomeGoalTrack(nowMs) && homeGoal.found));
    Serial.print('\t');
    Serial.print(boolText(homeGoalCloseEnough));
    Serial.print('\t');
    Serial.print(homeGoal.angle, 1);
    Serial.print('\t');
    Serial.print(homeGoal.area);
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
                const float incomingAngle = incomingData.substring(2).toFloat();
                latestRawBallTheta = incomingAngle;
                latestBallAngle = DriveHelpers::wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            } else if (!incomingData.startsWith("r ")) {
                const float incomingAngle = incomingData.toFloat();
                latestRawBallTheta = incomingAngle;
                latestBallAngle = DriveHelpers::wrapAngle180(180.0f - incomingAngle);
                lastBallReadMs = millis();
            }
        }
    }
}

bool tryLockHomeGoal(unsigned long timeoutMs) {
    if (!pixyAvailable || homeGoalSignature != 0) {
        return homeGoalSignature != 0;
    }

    return pixyLockGoalSignature(homeGoalSignature,
                                 PixySig::kYellowGoal,
                                 PixySig::kBlueGoal,
                                 timeoutMs,
                                 &lastHomeGoalSeenTime);
}

void recoverHomePosition(const PixyBlock& homeGoal, double turnCommand) {
    if (homeGoal.found) {
        const float goalAngle = getHomeGoalDriveAngle(homeGoal,
                                                      Constants::Ajolote::kHomeGoalCenterDeadbandDeg);
        lastHomeGoalDriveAngle = getHomeGoalDriveAngle(homeGoal, 0.0f);
        filteredRecoveryAngle = DriveHelpers::smoothAngleChange(
            filteredRecoveryAngle,
            goalAngle,
            Constants::Striker::kAngleSmoothingAlpha);
        robot.motors.move(filteredRecoveryAngle, kHomeRecoveryDrivePwm, turnCommand);
        return;
    }

    filteredRecoveryAngle = 0.0f;

    if (lastHomeGoalDriveAngle > 0.0f) {
        robot.motors.move(90.0f, kGoalSearchDrivePwm, turnCommand);
        return;
    }

    if (lastHomeGoalDriveAngle < 0.0f) {
        robot.motors.move(270.0f, kGoalSearchDrivePwm, turnCommand);
        return;
    }

    robot.motors.move(0.0f, 0.0f, turnCommand);
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
        Serial.println("Continuing without Pixy home-goal tracking.");
    }

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);
    phototransistor_sensors.SetAlternatingLateralMuxEnabled(true);

    robot.begin();
    robot.motors.stop();
    delay(500);

    const int imuStatus = ajoloteImu.begin(calib);
    if (imuStatus == 0) {
        imuAvailable = true;
        Serial.println("[SUCCESS] MPU6500 IMU connected!");
    } else {
        imuAvailable = false;
        Serial.print("ERROR: IMU not found. Error code: ");
        Serial.println(imuStatus);
        Serial.println("Continuing with frozen IMU heading.");
    }

    HWSERIAL.begin(9600);
    HWSERIAL.setTimeout(5);

    tryLockHomeGoal(Constants::Ajolote::kGoalCaptureTimeoutMs);

    startupYaw = imuAvailable ? ajoloteImu.getAngle() : 0.0;
    // startupYaw = robot.bno.GetBNOData();
    orbitHeadingPD.reset();
    avoidHeadingPD.reset();
    motorsEnableMs = millis() + Constants::Ajolote::kStartupHoldMs;
    Serial.println("Ajolote debug ready");
}

void loop() {
    const unsigned long nowMs = millis();

    if (nowMs < motorsEnableMs) {
        robot.motors.stop();
        return;
    }

    phototransistor_sensors.AdvanceLateralMuxCycle();
    updateBallAngle();
    const bool hasBall = (nowMs - lastBallReadMs) <= kPIDLookForBallTimeoutMs;
    const float chaseAngle = hasBall ? latestBallAngle : 0.0f;
    const double currentYaw = imuAvailable ? ajoloteImu.getAngle() : startupYaw;
    // const double currentYaw = robot.bno.GetBNOData();

    PixyBlock homeGoal = pixyNoGoalBlock();
    if (pixyAvailable) {
        if (homeGoalSignature == 0) {
            tryLockHomeGoal(0);
        }

        homeGoal = pixyReadLockedGoal(homeGoalSignature);
        if (homeGoal.found) {
            lastHomeGoalSeenTime = nowMs;
            lastHomeGoalDriveAngle = getHomeGoalDriveAngle(homeGoal, 0.0f);
        }
    }

    const bool homeGoalCloseEnough = pixyAvailable &&
        pixyIsGoalCloseEnough(homeGoal, Constants::Ajolote::kHomeGoalMinAreaThreshold);
    const bool needsHomeGoalRecovery = pixyAvailable &&
        (!homeGoal.found || !homeGoalCloseEnough);
    const double orbitTurnCommand = orbitHeadingPD.calculate(startupYaw, currentYaw, true);
    const double avoidTurnCommand = avoidHeadingPD.calculate(startupYaw, currentYaw, true);

    if (currentState == RobotState::AVOIDING_LINE) {
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         homeGoal,
                         homeGoalCloseEnough,
                         activeAvoidSide,
                         isBallInFrontFromRawTheta(latestRawBallTheta));
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::GET_BEHIND_BALL;
            activeAvoidSide = AvoidSide::NONE;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
            return;
        }
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
                         homeGoal,
                         homeGoalCloseEnough,
                         activeAvoidSide,
                         isBallInFrontFromRawTheta(latestRawBallTheta));
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        return;
    }

    activeAvoidSide = AvoidSide::NONE;

    if (pixyAvailable && !tryLockHomeGoal(0)) {
        ballFrontStartMs = 0;
        currentState = RobotState::WAITING_FOR_HOME_GOAL;
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         homeGoal,
                         false,
                         activeAvoidSide,
                         false);
        robot.motors.move(0.0f, 0.0f, orbitTurnCommand);
        return;
    }

    if (needsHomeGoalRecovery) {
        ballFrontStartMs = 0;
        currentState = RobotState::RECOVERING_HOME_GOAL;
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         homeGoal,
                         homeGoalCloseEnough,
                         activeAvoidSide,
                         false);
        recoverHomePosition(homeGoal, orbitTurnCommand);
        return;
    }

    filteredRecoveryAngle = 0.0f;

    if (!hasBall) {
        ballFrontStartMs = 0;
        currentState = RobotState::GET_BEHIND_BALL;
        printDebugStatus(nowMs,
                         currentYaw,
                         latestBallAngle,
                         chaseAngle,
                         homeGoal,
                         homeGoalCloseEnough,
                         activeAvoidSide,
                         false);
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
    const float chaseDrivePwm = (fabsf(ballAngle) > 90.0f) ? kBehindBallDrivePwm : kChaseDrivePwm;

    if (hasConfirmedFrontBall) {
        currentState = RobotState::CARRY_BALL_FORWARD;
        printDebugStatus(nowMs,
                         currentYaw,
                         ballAngle,
                         0.0f,
                         homeGoal,
                         homeGoalCloseEnough,
                         activeAvoidSide,
                         ballInFront);
        robot.motors.move(0.0f, kCarryDrivePwm, orbitTurnCommand);
        return;
    }

    currentState = RobotState::GET_BEHIND_BALL;
    printDebugStatus(nowMs,
                     currentYaw,
                     ballAngle,
                     chaseDriveAngle,
                     homeGoal,
                     homeGoalCloseEnough,
                     activeAvoidSide,
                     ballInFront);
    robot.motors.move(chaseDriveAngle, chaseDrivePwm, orbitTurnCommand);
}
