#include <Arduino.h>
#include <math.h>
#include "constants.h"
#include "PID.h"
#include "photo.h"
#include "pixyLib.h"
#include "robot.h"
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
    CHASE_BALL,
    AVOIDING_LINE
};

enum class AvoidSide {
    NONE,
    FRONT,
    LEFT,
    RIGHT
};

RobotState currentState = RobotState::CHASE_BALL;

const float kChaseDrivePwm = 0.52f * Constants::Motor::maxPWM;
const float kBehindBallDrivePwm = 0.55f * Constants::Motor::maxPWM;
const float kAvoidDrivePwm = 0.67f * Constants::Motor::maxPWM;
constexpr unsigned long kDebugPrintIntervalMs = 120;
constexpr bool kEnableDebugPrints = false;
constexpr bool kEnablePixy = false;
constexpr bool kEnableLineAvoidance = true;
constexpr float kHeadingKp = 1.5f;
constexpr float kHeadingKd = 0.10f;
constexpr float kMaxTurnPwm = 55.0f;
constexpr float kMinTurnPwm = 40.0f;
constexpr float kHeadingSettleBandDeg = 5.8f;
constexpr float kBehindBallOrbitAdjustGain = 0.50f;
constexpr float kBehindBallOrbitAdjustMaxDeg = 18.0f;

PID headingPD(
    kHeadingKp,
    0.0f,
    kHeadingKd,
    kMaxTurnPwm,
    kMinTurnPwm,
    kHeadingSettleBandDeg
);

double startupYaw = 0.0;
unsigned long motorsEnableMs = 0;
unsigned long lastBallReadMs = 0;
unsigned long avoidStartTime = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastDebugHeaderMs = 0;
bool pixyAvailable = false;
float latestBallAngle = 0.0f;
float latestRawBallTheta = 0.0f;
int escapeAngle = 0;
AvoidSide activeAvoidSide = AvoidSide::NONE;

}  // namespace

bool isBallInFrontFromRawTheta(float rawTheta) {
    return fabsf(rawTheta) <= Constants::Striker::kBallFrontToleranceDeg;
}

float mapRobotBallAngleToDriveAngle(float robotAngle) {
    // The UART ball angle uses 0 = front, +/-180 = behind.
    // The current drivetrain responds with front/back mirrored, so flip only
    // the front-back axis while keeping left/right unchanged.
    return DriveHelpers::wrapAngle180(180.0f - robotAngle);
}

float getBehindBallOrbitAngle(float ballAngle) {
    const float orbitDirection = (ballAngle >= 0.0f) ? 1.0f : -1.0f;
    const float rearHalfErrorDeg = fabsf(ballAngle) - 90.0f;
    const float orbitAdjustment = DriveHelpers::clampSymmetric(
        rearHalfErrorDeg * kBehindBallOrbitAdjustGain,
        kBehindBallOrbitAdjustMaxDeg
    );
    return DriveHelpers::wrapAngle180(ballAngle - orbitDirection * orbitAdjustment);
}

const char* boolText(bool value) {
    return value ? "true" : "false";
}

const char* stateName(RobotState state) {
    switch (state) {
        case RobotState::AVOIDING_LINE:
            return "avoid";
        case RobotState::CHASE_BALL:
        default:
            return "chase";
    }
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

void printTriggeredLineSides() {
    if (!kEnableDebugPrints) {
        return;
    }

    if (phototransistor_sensors.HasLineOnSide(Side::Left)) {
        Serial.println("LINE on LEFT");
    }
    if (phototransistor_sensors.HasLineOnSide(Side::Right)) {
        Serial.println("LINE on RIGHT");
    }
    if (phototransistor_sensors.HasLineOnSide(Side::Front)) {
        Serial.println("LINE on FRONT");
    }
}

AvoidSide avoidSideFromEscapeAngle(int detectedEscapeAngle) {
    switch (detectedEscapeAngle) {
        case 180:
            return AvoidSide::FRONT;
        case 90:
            return AvoidSide::LEFT;
        case -90:
            return AvoidSide::RIGHT;
        default:
            return AvoidSide::NONE;
    }
}

void printDebugStatus(unsigned long nowMs,
                      RobotState state,
                      double currentYaw,
                      float rawTheta,
                      float ballAngle,
                      float chaseAngle,
                      float turnCommand,
                      AvoidSide avoidSide,
                      bool ballInFront,
                      bool hasBall) {
    if (!kEnableDebugPrints) {
        return;
    }

    if (nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
        return;
    }

    if (lastDebugHeaderMs == 0 || nowMs - lastDebugHeaderMs >= 2000) {
        Serial.println("state\tyaw\traw\tball\tchase\tturn\tline_side\tfront\thas_ball");
        lastDebugHeaderMs = nowMs;
    }

    lastDebugPrintMs = nowMs;
    Serial.print(stateName(state));
    Serial.print('\t');
    Serial.print(currentYaw, 1);
    Serial.print('\t');
    Serial.print(rawTheta, 1);
    Serial.print('\t');
    Serial.print(ballAngle, 1);
    Serial.print('\t');
    Serial.print(chaseAngle, 1);
    Serial.print('\t');
    Serial.print(turnCommand, 1);
    Serial.print('\t');
    Serial.print(avoidSideName(avoidSide));
    Serial.print('\t');
    Serial.print(boolText(ballInFront));
    Serial.print('\t');
    Serial.println(boolText(hasBall));
}

void updateBallAngle() {
    if (HWSERIAL.available() > 0) {
        String incomingData = HWSERIAL.readStringUntil('\n');
        incomingData.trim();

        if (incomingData.length() > 0) {
            const float incomingAngle = incomingData.toFloat();
            latestRawBallTheta = incomingAngle;
            latestBallAngle = incomingAngle;
            lastBallReadMs = millis();
        }
    }
}

void initializePixy() {
    if (!kEnablePixy) {
        pixyAvailable = false;
        Serial.println("Colibri Pixy disabled");
        return;
    }

    Serial.println("Starting I2C bus for Pixy...");
    const int8_t pixyStatus = pixy.init();
    if (pixyStatus == 0) {
        pixyAvailable = true;
        Serial.println("[SUCCESS] Pixy2 connected!");
    } else {
        pixyAvailable = false;
        Serial.print("ERROR: Pixy not found. Error code: ");
        Serial.println(pixyStatus);
        Serial.println("Continuing without Pixy.");
    }
}

void setup() {
    Serial.begin(115200);
    initializePixy();
    HWSERIAL.begin(Constants::kIRSerialBaud);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    startupYaw = robot.bno.GetBNOData();
    headingPD.reset();

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    const unsigned long setupNowMs = millis();
    motorsEnableMs = setupNowMs + Constants::Striker::kStartupHoldMs;

    Serial.println("Colibri...ready to fly");
}

void loop() {
    const unsigned long nowMs = millis();

    if (nowMs < motorsEnableMs) {
        robot.motors.stop();
        return;
    }

    updateBallAngle();

    const double currentYaw = robot.bno.GetBNOData();
    const float turnCommand = static_cast<float>(headingPD.calculate(startupYaw, currentYaw, true));
    const bool hasBall = lastBallReadMs != 0;

    if (kEnableLineAvoidance && currentState == RobotState::AVOIDING_LINE) {
        printDebugStatus(nowMs,
                         currentState,
                         currentYaw,
                         latestRawBallTheta,
                         latestBallAngle,
                         static_cast<float>(escapeAngle),
                         turnCommand,
                         activeAvoidSide,
                         false,
                         hasBall);
        if (nowMs - avoidStartTime >= Constants::kAvoidDurationMs) {
            currentState = RobotState::CHASE_BALL;
            activeAvoidSide = AvoidSide::NONE;
        } else {
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, turnCommand);
        }
        return;
    }

    if (kEnableLineAvoidance) {
        const int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
        if (escapeAngleDetected != -1) {
            escapeAngle = escapeAngleDetected;
            avoidStartTime = nowMs;
            currentState = RobotState::AVOIDING_LINE;
            activeAvoidSide = avoidSideFromEscapeAngle(escapeAngleDetected);
            printTriggeredLineSides();
            if (kEnableDebugPrints) {
                Serial.print("Line detected, escaping at ");
                Serial.println(escapeAngle);
            }
            printDebugStatus(nowMs,
                             currentState,
                             currentYaw,
                             latestRawBallTheta,
                             latestBallAngle,
                             static_cast<float>(escapeAngle),
                             turnCommand,
                             activeAvoidSide,
                             false,
                             hasBall);
            robot.motors.move(static_cast<float>(escapeAngle), kAvoidDrivePwm, turnCommand);
            return;
        }
    }

    activeAvoidSide = AvoidSide::NONE;

    const float chaseAngle = latestBallAngle;
    const bool shouldOrbitBehindBall = fabsf(chaseAngle) > 90.0f;
    const float robotMotionAngle = shouldOrbitBehindBall
        ? getBehindBallOrbitAngle(chaseAngle)
        : chaseAngle;
    const float driveAngle = mapRobotBallAngleToDriveAngle(robotMotionAngle);
    const float chaseDrivePwm = shouldOrbitBehindBall ? kBehindBallDrivePwm : kChaseDrivePwm;
    const bool ballInFront = isBallInFrontFromRawTheta(latestRawBallTheta);

    printDebugStatus(nowMs,
                     currentState,
                     currentYaw,
                     latestRawBallTheta,
                     chaseAngle,
                     driveAngle,
                     turnCommand,
                     activeAvoidSide,
                     ballInFront,
                     true);
    robot.motors.move(driveAngle, chaseDrivePwm, turnCommand);
}
