#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "photo.h"
#include "robot.h"
#include "pixyLib.h"

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

enum class RobotState {
    LOOKING_FOR_BALL,
    DRIVING_TO_GOAL,
    AVOIDING_LINE
};

RobotState current_state = RobotState::LOOKING_FOR_BALL;
RobotState state_before_avoid = RobotState::LOOKING_FOR_BALL;

double targetYaw = 0.0;
unsigned long avoid_start_time = 0;
unsigned long last_opponent_goal_seen_time = 0;
int escapeAngle = 0;
uint16_t opponent_goal_signature = 0;
float filtered_drive_angle = 0.0f;

const float goalDrivePwm = Constants::Striker::kGoalDrivePwmRatio * Constants::Motor::maxPWM;
const float avoidDrivePwm = Constants::Striker::kAvoidDrivePwmRatio * Constants::Motor::maxPWM;
const float chaseDrivePwm = Constants::Striker::kChaseDrivePwmRatio * Constants::Motor::maxPWM;

PID headingPD(
    Constants::Striker::kHeadingKp,
    0.0f,
    Constants::Striker::kHeadingKd,
    Constants::Striker::kMaxTurnPwm,
    Constants::Striker::kMinTurnPwm,
    Constants::Striker::kHeadingSettleBandDeg
);

float smoothAngleChange(float previous, float current) {
    return previous + (current - previous) * Constants::Striker::kAngleSmoothingAlpha;
}

double wrapAngle180(double angleDegrees) {
    angleDegrees = fmod(angleDegrees + 180.0, 360.0);
    if (angleDegrees < 0.0) {
        angleDegrees += 360.0;
    }
    return angleDegrees - 180.0;
}

bool isGoalCentered(const PixyBlock& goal) {
    if (!goal.found) {
        return false;
    }

    return abs(static_cast<int>(goal.x) - static_cast<int>(PixyFrame::kCenterX)) <=
           Constants::Striker::kGoalCenterTolerancePx;
}

bool isGoalCloseEnoughToAim(const PixyBlock& goal) {
    return goal.found && goal.area >= Constants::Striker::kGoalAimAreaThreshold;
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
    pixyLockGoalSignature(opponent_goal_signature,
                          PixySig::kYellowGoal,
                          PixySig::kBlueGoal,
                          Constants::Striker::kGoalCaptureTimeoutMs,
                          &last_opponent_goal_seen_time,
                          "opponent goal");

    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();
}

void loop() {
    const double yaw = robot.bno.GetBNOData();
    const float holdAngle = 0.0f;
    const float holdDrivePwm = 0.0f;

    if (current_state != RobotState::AVOIDING_LINE) {
        const int detectedEscapeAngle = phototransistor_sensors.CheckPhotosOnField();
        if (detectedEscapeAngle != -1) {
            state_before_avoid = current_state;
            escapeAngle = detectedEscapeAngle;
            avoid_start_time = millis();
            current_state = RobotState::AVOIDING_LINE;
            filtered_drive_angle = 0.0f;
        }
    }

    const double avoidTurnCommand = headingPD.calculate(targetYaw, yaw, true);
    if (current_state == RobotState::AVOIDING_LINE) {
        if (millis() - avoid_start_time >= Constants::kAvoidDurationMs) {
            robot.motors.move(holdAngle, holdDrivePwm, avoidTurnCommand);
            current_state = state_before_avoid;
        } else {
            robot.motors.move(escapeAngle, avoidDrivePwm, avoidTurnCommand);
        }
        return;
    }

    if (opponent_goal_signature == 0) {
        pixyLockGoalSignature(opponent_goal_signature,
                              PixySig::kYellowGoal,
                              PixySig::kBlueGoal,
                              0,
                              &last_opponent_goal_seen_time,
                              "opponent goal");
    }

    const PixyBlock opponentGoal = pixyReadLockedGoal(opponent_goal_signature);
    if (opponentGoal.found) {
        last_opponent_goal_seen_time = millis();
    }

    robot.irring.UpdateData();
    const double irStrength = robot.irring.GetStrength();
    const float rawIrBallAngle = static_cast<float>(robot.irring.GetAngle(
        Constants::Striker::kIRBallFollowOffsetBack,
        Constants::Striker::kIRBallFollowOffsetSide,
        Constants::Striker::kIRBallFollowOffsetFront));
    const bool hasFarBall = irStrength >= Constants::Striker::kIRFarBallStrength;
    const bool hasCloseBall = irStrength >= Constants::Striker::kIRCloseBallStrength;
    const bool goalCentered = isGoalCentered(opponentGoal);
    const bool goalCloseEnoughToAim = isGoalCloseEnoughToAim(opponentGoal);

    if (current_state == RobotState::LOOKING_FOR_BALL) {
        targetYaw = yaw;
        const double turnCommand = headingPD.calculate(targetYaw, yaw, true);

        if (hasCloseBall) {
            current_state = RobotState::DRIVING_TO_GOAL;
            filtered_drive_angle = 0.0f;
            headingPD.reset();
            robot.motors.move(0.0f, goalDrivePwm, 0.0f);
            return;
        }

        if (hasFarBall) {
            const float irBallAngle = constrain(rawIrBallAngle,
                                                -Constants::Striker::kIRBallAngleClampDeg,
                                                Constants::Striker::kIRBallAngleClampDeg);
            filtered_drive_angle = smoothAngleChange(filtered_drive_angle, irBallAngle);
            robot.motors.move(filtered_drive_angle, chaseDrivePwm, turnCommand);
            return;
        }

        robot.motors.move(holdAngle, holdDrivePwm, turnCommand);
        return;
    }

    if (!hasCloseBall) {
        current_state = RobotState::LOOKING_FOR_BALL;
        filtered_drive_angle = 0.0f;
        targetYaw = yaw;
        headingPD.reset();

        if (hasFarBall) {
            const double turnCommand = headingPD.calculate(targetYaw, yaw, true);
            const float irBallAngle = constrain(rawIrBallAngle,
                                                -Constants::Striker::kIRBallAngleClampDeg,
                                                Constants::Striker::kIRBallAngleClampDeg);
            filtered_drive_angle = smoothAngleChange(filtered_drive_angle, irBallAngle);
            robot.motors.move(filtered_drive_angle, chaseDrivePwm, turnCommand);
            return;
        }
        return;
    }

    const bool lostOpponentGoal =
        opponent_goal_signature == 0 ||
        (!opponentGoal.found &&
         millis() - last_opponent_goal_seen_time >= Constants::Striker::kGoalLostTimeoutMs);

    if (lostOpponentGoal) {
        targetYaw = yaw;
        const double turnCommand = headingPD.calculate(targetYaw, yaw, true);
        robot.motors.move(0.0f, goalDrivePwm, turnCommand);
        filtered_drive_angle = 0.0f;
        return;
    }

    if (goalCloseEnoughToAim || goalCentered) {
        targetYaw = wrapAngle180(yaw + opponentGoal.angle);
    } else {
        targetYaw = yaw;
    }

    const double turnCommand = headingPD.calculate(targetYaw, yaw, true);
    robot.motors.move(0.0f, goalDrivePwm, turnCommand);
}
