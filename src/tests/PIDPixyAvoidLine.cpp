#include <Arduino.h>
#include "constants.h"
#include "photo.h"
#include "robot.h"
#include "pixyLib.h"

Robot robot;
Pixy2 pixy;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

double targetYaw = 0.0;
double lastHeadingError = 0.0;
unsigned long lastControlTimeMs = 0;
unsigned long lastDebugTimeMs = 0;
unsigned long lastPixyTimeMs = 0;
unsigned long avoidStartTimeMs = 0;
unsigned long searchPhaseStartTimeMs = 0;
unsigned long activeAvoidDurationMs = 0;
unsigned long startupTimeMs = 0;

bool ballVisible = false;
bool searchRight = true;
double ballDriveAngle = 0.0;
int escapeAngle = 0;

const float kHeadingKp = 2.3f;
const float kHeadingKd = 0.18f;
const float kMaxTurnPwm = 90.0f;
const float kMinTurnPwm = 45.0f;
const float kHeadingSettleBandDeg = 4.0f;
const float kDrivePwm = 0.40f * Constants::Motor::maxPWM;
const float kSearchDrivePwm = 0.30f * Constants::Motor::maxPWM;
const float kFrontAvoidDrivePwm = 0.45f * Constants::Motor::maxPWM;
const unsigned long kPixyUpdateIntervalMs = 60;
const unsigned long kDebugIntervalMs = 120;
const unsigned long kAvoidDurationMs = 350;
const unsigned long kFrontAvoidDurationMs = 450;
const unsigned long kSearchSwitchIntervalMs = 700;
const unsigned long kStartupIgnoreLineMs = 1200;
const uint8_t kBallSignature = 1;
const uint8_t kBaselineSamples = 20;
const uint16_t kBaselineDelayMs = 10;

const uint16_t kLeftMargins[Constants::kPhotoLeftElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kRightMargins[Constants::kPhotoRightElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kFrontMargins[Constants::kPhotoFrontElements] = {
    15, 15, 15, 15, 15, 15
};

enum class RobotState {
    TRACKING_BALL,
    AVOIDING_LINE
};

RobotState currentState = RobotState::TRACKING_BALL;

static double WrapAngle180(double deg) {
    while (deg > 180.0) {
        deg -= 360.0;
    }
    while (deg < -180.0) {
        deg += 360.0;
    }
    return deg;
}

static double ClampMagnitude(double value, double maxMagnitude) {
    if (value > maxMagnitude) {
        return maxMagnitude;
    }
    if (value < -maxMagnitude) {
        return -maxMagnitude;
    }
    return value;
}

static double ComputeHeadingTurnCommand(double yaw, unsigned long now) {
    double deltaTime = (now - lastControlTimeMs) / 1000.0;
    if (deltaTime <= 0.0) {
        deltaTime = 0.001;
    }

    double headingError = WrapAngle180(targetYaw - yaw);
    double headingDerivative = (headingError - lastHeadingError) / deltaTime;
    double turnCommand = (kHeadingKp * headingError) + (kHeadingKd * headingDerivative);
    turnCommand = ClampMagnitude(turnCommand, kMaxTurnPwm);

    if (fabs(headingError) <= kHeadingSettleBandDeg) {
        turnCommand = 0.0;
    } else if (fabs(turnCommand) < kMinTurnPwm) {
        turnCommand = (turnCommand >= 0.0) ? kMinTurnPwm : -kMinTurnPwm;
    }

    lastHeadingError = headingError;
    lastControlTimeMs = now;

    return turnCommand;
}

static void UpdatePixyBallTracking(unsigned long now) {
    if (now - lastPixyTimeMs < kPixyUpdateIntervalMs) {
        return;
    }

    lastPixyTimeMs = now;
    pixy.ccc.getBlocks();

    ballVisible = false;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature != kBallSignature) {
            continue;
        }

        ballVisible = true;
        uint16_t xComponent = pixy.ccc.blocks[i].m_x;
        uint16_t yComponent = pixy.ccc.blocks[i].m_y;
        ballDriveAngle = (atan2((double)yComponent, (double)xComponent) * 180.0 / PI) - 45.0;
        break;
    }
}

static int GetEscapeAngle() {
    PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);
    PhotoData left = phototransistor_sensors.CheckPhotosOnField(Side::Left);
    PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);

    if (front.is_on_line) {
        return 180;
    }
    if (left.is_on_line) {
        return 270;
    }
    if (right.is_on_line) {
        return 90;
    }
    return -1;
}

static float GetSearchAngle(unsigned long now) {
    if (now - searchPhaseStartTimeMs >= kSearchSwitchIntervalMs) {
        searchPhaseStartTimeMs = now;

        searchRight = !searchRight;
    }

    return searchRight ? 90.0f : 270.0f;
}

void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.motors.stop();
    delay(2000);

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetMargins(Side::Left, kLeftMargins, Constants::kPhotoLeftElements);
    phototransistor_sensors.SetMargins(Side::Right, kRightMargins, Constants::kPhotoRightElements);
    phototransistor_sensors.SetMargins(Side::Front, kFrontMargins, Constants::kPhotoFrontElements);
    phototransistor_sensors.SetThresholdPadding(5);
    phototransistor_sensors.CaptureBaseline(kBaselineSamples, kBaselineDelayMs);

    int result = pixy.init();
    if (result != 0) {
        Serial.println("[FAIL] Pixy2 not found. Check wiring or power");
        while (true) {
        }
    }

    targetYaw = robot.imu.getAngle();
    lastControlTimeMs = millis();
    searchPhaseStartTimeMs = millis();
    startupTimeMs = millis();

}

void loop() {
    unsigned long now = millis();
    double yaw = robot.imu.getAngle();
    double turnCommand = ComputeHeadingTurnCommand(yaw, now);

    if (now - startupTimeMs >= kStartupIgnoreLineMs) {
        int detectedEscapeAngle = GetEscapeAngle();
        if (detectedEscapeAngle != -1) {
            escapeAngle = detectedEscapeAngle;
            activeAvoidDurationMs = (escapeAngle == 180) ? kFrontAvoidDurationMs : kAvoidDurationMs;
            avoidStartTimeMs = now;
            currentState = RobotState::AVOIDING_LINE;
        }
    }

    if (currentState == RobotState::AVOIDING_LINE) {
        if (now - avoidStartTimeMs >= activeAvoidDurationMs) {
            currentState = RobotState::TRACKING_BALL;
        } else {
            const float avoidDrivePwm = (escapeAngle == 180) ? kFrontAvoidDrivePwm : kDrivePwm;
            robot.motors.move(escapeAngle, avoidDrivePwm, turnCommand);
            return;
        }
    }

    UpdatePixyBallTracking(now);

    float driveAngle = ballVisible ? (float)ballDriveAngle : GetSearchAngle(now);
    float drivePwm = ballVisible ? kDrivePwm : kSearchDrivePwm;

    robot.motors.move(driveAngle, drivePwm, turnCommand);

}
