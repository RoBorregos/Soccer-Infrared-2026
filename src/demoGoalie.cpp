#include <Arduino.h>
#include <math.h>
#include "photo.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"
#include "pixyLib.h"

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

enum class RobotState {
    RETREAT_TO_GOAL,
    DEFEND_GOAL,
    INTERCEPT_BALL,
    AVOID_LINE
};

RobotState current_state = RobotState::RETREAT_TO_GOAL;
RobotState state_before_avoid = RobotState::RETREAT_TO_GOAL;

unsigned long retreat_start_time = 0;
unsigned long avoid_start_time = 0;
unsigned long last_home_goal_seen_time = 0;
int escapeAngle = 0;
double targetYaw = 0.0;
uint16_t home_goal_signature = 0;
float filtered_drive_angle = 0.0f;
float last_home_goal_angle = 0.0f;

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

float smoothAngleChange(float previous, float current) {
    return previous + (current - previous) * Constants::Striker::kAngleSmoothingAlpha;
}

float clampSymmetric(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

bool isGoalCentered(const PixyBlock& goal) {
    if (!goal.found) {
        return false;
    }

    const int centerOffset = static_cast<int>(goal.x) - static_cast<int>(PixyFrame::kCenterX);
    return abs(centerOffset) <= Constants::kGoalKeeperTresholdX;
}

bool isGoalInsideLane(const PixyBlock& goal) {
    return goal.found &&
           goal.x >= Constants::kLeftGoalKeeperTresholdX &&
           goal.x <= Constants::kRightGoalKeeperTresholdX;
}

float goalCorrectionAngle(const PixyBlock& goal) {
    return pixyGetGoalDriveAngle(goal,
                                 Constants::Goalie::kGoalAngleClampDeg,
                                 Constants::Goalie::kGoalTrackDirectionSign,
                                 Constants::Goalie::kGoalCenterDeadbandDeg);
}

bool isBallThreat(double strength, float angle) {
    return strength >= Constants::Goalie::kIRCloseBallStrength &&
           fabs(angle) <= Constants::Goalie::kIRThreatAngleToleranceDeg;
}

float makeInterceptAngle(float goalAngle, float ballAngle) {
    const float clampedBallAngle = clampSymmetric(ballAngle, Constants::Goalie::kBallAngleClampDeg);
    float driveAngle = (clampedBallAngle * Constants::Goalie::kBallFollowWeight) +
                       (goalAngle * Constants::Goalie::kGoalCorrectionWeight);
    driveAngle = clampSymmetric(driveAngle, Constants::Goalie::kBallAngleClampDeg);

    if (fabs(driveAngle) <= Constants::Goalie::kInterceptDeadbandDeg) {
        return 0.0f;
    }

    return driveAngle;
}

void recoverHomePosition(const PixyBlock& homeGoal, double turnCommand) {
    if (homeGoal.found) {
        const float goalAngle = goalCorrectionAngle(homeGoal);
        last_home_goal_angle = goalAngle;
        filtered_drive_angle = smoothAngleChange(filtered_drive_angle, goalAngle);
        robot.motors.move(filtered_drive_angle, retreatDrivePwm, turnCommand);
        return;
    }

    filtered_drive_angle = 0.0f;

    if (last_home_goal_angle > 0.0f) {
        robot.motors.move(90.0f, goalSearchDrivePwm, turnCommand);
        return;
    }

    if (last_home_goal_angle < 0.0f) {
        robot.motors.move(270.0f, goalSearchDrivePwm, turnCommand);
        return;
    }

    robot.motors.stop();
}

void setup() {
    Serial.begin(115200);

    robot.begin();
    robot.motors.stop();
    delay(2000);

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);

    pixyInit();
    pixyLockGoalSignature(home_goal_signature,
                          PixySig::kYellowGoal,
                          PixySig::kBlueGoal,
                          Constants::Goalie::kGoalCaptureTimeoutMs,
                          &last_home_goal_seen_time,
                          "home goal");

    targetYaw = robot.imu.getAngle();
    headingPD.reset();
    retreat_start_time = millis();
}

void loop() {
    const double currentYaw = robot.imu.getAngle();
    const double turnCommand = headingPD.calculate(targetYaw, currentYaw, true);
    const float holdAngle = 0.0f;
    const float holdDrivePwm = 0.0f;

    if (current_state != RobotState::AVOID_LINE) {
        const int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
        if (escapeAngleDetected != -1) {
            state_before_avoid = current_state;
            escapeAngle = escapeAngleDetected;
            avoid_start_time = millis();
            current_state = RobotState::AVOID_LINE;
            robot.motors.move(escapeAngle, defenseDrivePwm, turnCommand);
            return;
        }
    }

    if (current_state == RobotState::AVOID_LINE) {
        if (millis() - avoid_start_time >= Constants::kAvoidDurationMs) {
            robot.motors.move(holdAngle, holdDrivePwm, turnCommand);
            current_state = state_before_avoid;
        } else {
            robot.motors.move(escapeAngle, defenseDrivePwm, turnCommand);
        }
        return;
    }

    if (home_goal_signature == 0) {
        pixyLockGoalSignature(home_goal_signature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              0,
                              &last_home_goal_seen_time,
                              "home goal");
    }

    const PixyBlock homeGoal = pixyReadLockedGoal(home_goal_signature);
    if (homeGoal.found) {
        last_home_goal_seen_time = millis();
        last_home_goal_angle = goalCorrectionAngle(homeGoal);
    }

    if (current_state == RobotState::RETREAT_TO_GOAL) {
        const bool retreatTimeElapsed =
            millis() - retreat_start_time >= Constants::Goalie::kRetreatDurationMs;

        if (!retreatTimeElapsed || !homeGoal.found) {
            recoverHomePosition(homeGoal, turnCommand);
            return;
        }

        current_state = RobotState::DEFEND_GOAL;
        filtered_drive_angle = 0.0f;
    }

    const bool lostHomeGoal =
        home_goal_signature == 0 ||
        (!homeGoal.found && millis() - last_home_goal_seen_time >= Constants::Goalie::kGoalLostTimeoutMs);

    if (lostHomeGoal) {
        current_state = RobotState::RETREAT_TO_GOAL;
        retreat_start_time = millis();
        recoverHomePosition(homeGoal, turnCommand);
        return;
    }

    const float goalAngle = goalCorrectionAngle(homeGoal);
    const bool goalOutsideLane = !isGoalInsideLane(homeGoal);
    const bool goalNeedsCentering = !isGoalCentered(homeGoal);

    // Future IR logic will live here.
    // Example skeleton:
    // robot.irring.UpdateData();
    // const double irRadius = robot.irring.GetStrength();
    // const float irBallAngle = static_cast<float>(robot.irring.GetAngle(...));
    // const bool ballThreat = isBallThreat(irRadius, irBallAngle);

    if (goalOutsideLane || goalNeedsCentering) {
        current_state = RobotState::DEFEND_GOAL;
        filtered_drive_angle = smoothAngleChange(filtered_drive_angle, goalAngle);
        robot.motors.move(filtered_drive_angle, homeDrivePwm, turnCommand);
        return;
    }

    // Future IR-based intercept logic should move the goalie into INTERCEPT_BALL.
    // Example:
    // if (ballThreat) {
    //     current_state = RobotState::INTERCEPT_BALL;
    //     const float interceptAngle = makeInterceptAngle(goalAngle, irBallAngle);
    //     filtered_drive_angle = smoothAngleChange(filtered_drive_angle, interceptAngle);
    //     robot.motors.move(filtered_drive_angle, interceptDrivePwm, turnCommand);
    //     return;
    // }

    current_state = RobotState::DEFEND_GOAL;

    if (goalAngle != 0.0f) {
        filtered_drive_angle = smoothAngleChange(filtered_drive_angle, goalAngle);
        robot.motors.move(filtered_drive_angle, defenseDrivePwm, turnCommand);
        return;
    }

    filtered_drive_angle = 0.0f;
    robot.motors.move(holdAngle, holdDrivePwm, turnCommand);
}
