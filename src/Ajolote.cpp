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
    WAITING_FOR_HOME_GOAL,
    RECOVERING_HOME_GOAL,
    CHASING_BALL,
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

const float kChaseDrivePwm = Constants::Ajolote::kChaseDrivePwmRatio * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = Constants::Ajolote::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float kHomeRecoveryDrivePwm = Constants::Ajolote::kHomeRecoveryDrivePwmRatio * Constants::Motor::maxPWM;
const float kGoalSearchDrivePwm = Constants::Ajolote::kGoalSearchDrivePwmRatio * Constants::Motor::maxPWM;
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
unsigned long lastHomeGoalSeenTime = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
uint16_t homeGoalSignature = 0;
bool pixyAvailable = false;
float latestBallAngle = 0.0f;
float filteredRecoveryAngle = 0.0f;
float lastHomeGoalAngle = 0.0f;
int escapeAngle = 0;
AvoidSide activeAvoidSide = AvoidSide::NONE;

}

bool hasFreshHomeGoalTrack(unsigned long nowMs) {
    return homeGoalSignature != 0 &&
           (nowMs - lastHomeGoalSeenTime) <= Constants::Ajolote::kGoalLostTimeoutMs;
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
        case RobotState::CHASING_BALL:
            return "chase_ball";
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
                      float ballAngle,
                      const PixyBlock& homeGoal,
                      bool homeGoalCloseEnough) {
    if (nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
        return;
    }

    lastDebugPrintMs = nowMs;
    Serial.print("state=");
    Serial.print(stateName(currentState));
    Serial.print(" goal_locked=");
    Serial.print(boolText(homeGoalSignature != 0));
    Serial.print(" goal_fresh=");
    Serial.print(boolText(hasFreshHomeGoalTrack(nowMs)));
    Serial.print(" goal_big_enough=");
    Serial.print(boolText(homeGoalCloseEnough));
    Serial.print(" goal_area=");
    Serial.print(homeGoal.area);
    Serial.print(" goal_angle=");
    Serial.print(homeGoal.angle, 1);
    Serial.print(" ball_angle=");
    Serial.print(ballAngle, 1);
    Serial.print(" avoiding_line=");
    Serial.print(boolText(activeAvoidSide != AvoidSide::NONE));
    Serial.print(" line_side=");
    Serial.println(avoidSideName(activeAvoidSide));
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

        // Mirror the UART ball angle exactly like Colibri so Ajolote keeps the
        // same full-angle chase behavior while using the heading PID the whole time.
        latestBallAngle = DriveHelpers::wrapAngle180(180.0f - incomingData.toFloat());
        lastBallReadMs = millis();
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

float getHomeGoalDriveAngle(const PixyBlock& homeGoal, float deadbandDeg) {
    return pixyGetGoalDriveAngle(homeGoal,
                                 Constants::Ajolote::kHomeGoalAngleClampDeg,
                                 Constants::Goalie::kGoalTrackDirectionSign,
                                 deadbandDeg);
}

void recoverHomePosition(const PixyBlock& homeGoal, double turnCommand) {
    if (homeGoal.found) {
        // Ajolote keeps the locked home goal as the only retreat reference.
        const float goalAngle = getHomeGoalDriveAngle(homeGoal,
                                                      Constants::Ajolote::kHomeGoalCenterDeadbandDeg);
        lastHomeGoalAngle = getHomeGoalDriveAngle(homeGoal, 0.0f);
        filteredRecoveryAngle = DriveHelpers::smoothAngleChange(
            filteredRecoveryAngle,
            goalAngle,
            Constants::Striker::kAngleSmoothingAlpha);
        robot.motors.move(filteredRecoveryAngle, kHomeRecoveryDrivePwm, turnCommand);
        return;
    }

    filteredRecoveryAngle = 0.0f;

    if (lastHomeGoalAngle > 0.0f) {
        robot.motors.move(90.0f, kGoalSearchDrivePwm, turnCommand);
        return;
    }

    if (lastHomeGoalAngle < 0.0f) {
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

    robot.begin();
    robot.motors.stop();
    delay(2000);

    HWSERIAL.begin(9600);
    HWSERIAL.setTimeout(5);

    // Lock onto SIG2 or SIG3 once and never switch goals after boot.
    tryLockHomeGoal(Constants::Ajolote::kGoalCaptureTimeoutMs);

    startupYaw = robot.bno.GetBNOData();
    chaseHeadingPD.reset();
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

    const double currentYaw = robot.bno.GetBNOData();
    const double chaseTurnCommand = chaseHeadingPD.calculate(startupYaw, currentYaw, true);
    const double avoidTurnCommand = avoidHeadingPD.calculate(startupYaw, currentYaw, true);

    if (currentState == RobotState::AVOIDING_LINE) {
        const PixyBlock emptyGoal = pixyNoGoalBlock();
        printDebugStatus(nowMs, latestBallAngle, emptyGoal, false);
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::CHASING_BALL;
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
        const PixyBlock emptyGoal = pixyNoGoalBlock();
        printDebugStatus(nowMs, latestBallAngle, emptyGoal, false);
        robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, avoidTurnCommand);
        return;
    }

    activeAvoidSide = AvoidSide::NONE;
    updateBallAngle();

    if (pixyAvailable && !tryLockHomeGoal(0)) {
        currentState = RobotState::WAITING_FOR_HOME_GOAL;
        const PixyBlock emptyGoal = pixyNoGoalBlock();
        printDebugStatus(nowMs, latestBallAngle, emptyGoal, false);
        robot.motors.move(0.0f, 0.0f, chaseTurnCommand);
        return;
    }

    PixyBlock homeGoal = pixyNoGoalBlock();
    if (pixyAvailable) {
        homeGoal = pixyReadLockedGoal(homeGoalSignature);
        if (homeGoal.found) {
            lastHomeGoalSeenTime = nowMs;
            lastHomeGoalAngle = getHomeGoalDriveAngle(homeGoal, 0.0f);
        }
    }

    const bool homeGoalCloseEnough = pixyAvailable &&
        pixyIsGoalCloseEnough(homeGoal, Constants::Ajolote::kHomeGoalMinAreaThreshold);
    const bool needsHomeGoalRecovery = pixyAvailable &&
        (!hasFreshHomeGoalTrack(nowMs) || (homeGoal.found && !homeGoalCloseEnough));

    // Goal safety always wins over ball attack so Ajolote does not wander too far.
    if (needsHomeGoalRecovery) {
        currentState = RobotState::RECOVERING_HOME_GOAL;
        printDebugStatus(nowMs, latestBallAngle, homeGoal, homeGoalCloseEnough);
        recoverHomePosition(homeGoal, chaseTurnCommand);
        return;
    }

    filteredRecoveryAngle = 0.0f;

    if (nowMs - lastBallReadMs > Constants::kIRFreshDataTimeoutMs) {
        currentState = RobotState::CHASING_BALL;
        printDebugStatus(nowMs, latestBallAngle, homeGoal, homeGoalCloseEnough);
        robot.motors.move(0.0f, 0.0f, chaseTurnCommand);
        return;
    }

    currentState = RobotState::CHASING_BALL;
    printDebugStatus(nowMs, latestBallAngle, homeGoal, homeGoalCloseEnough);
    robot.motors.move(latestBallAngle, kChaseDrivePwm, chaseTurnCommand);
}
