#include <Arduino.h>
#include "photo.h"
#include "constants.h"
#include "robot.h"

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

const float drivePwm = 0.45f * Constants::Motor::maxPWM;
const unsigned long kAvoidDurationMs = 350;
const uint8_t kBaselineSamples = 20;
const uint16_t kBaselineDelayMs = 10;

const uint16_t kLeftMargins[Constants::kPhotoLeftElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kRightMargins[Constants::kPhotoRightElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kFrontMargins[Constants::kPhotoFrontElements] = {
    40, 40, 40, 40, 40, 40
};

enum class RobotState { IDLE, AVOIDING_LINE };
RobotState current_state = RobotState::IDLE;

unsigned long avoid_start_time = 0;
int escapeAngle = 0;

void setup()
{
    robot.begin();
    robot.motors.stop();
    delay(2000);

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetMargins(Side::Left, kLeftMargins, Constants::kPhotoLeftElements);
    phototransistor_sensors.SetMargins(Side::Right, kRightMargins, Constants::kPhotoRightElements);
    phototransistor_sensors.SetMargins(Side::Front, kFrontMargins, Constants::kPhotoFrontElements);
    phototransistor_sensors.SetRequiredConfirmations(1);
    phototransistor_sensors.SetThresholdPadding(5);
    phototransistor_sensors.CaptureBaseline(kBaselineSamples, kBaselineDelayMs);
}

void loop()
{
    if (current_state == RobotState::AVOIDING_LINE)
    {
        if (millis() - avoid_start_time >= kAvoidDurationMs)
        {
            robot.motors.stop();
            current_state = RobotState::IDLE;
        }
        else
        {
            robot.motors.move(escapeAngle, drivePwm);
        }
        return;
    }

    PhotoData left = phototransistor_sensors.CheckPhotosOnField(Side::Left);
    PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);
    PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);

    if (front.is_on_line || left.is_on_line || right.is_on_line)
    {
        if (front.is_on_line)
        {
            escapeAngle = 180;
        }
        else if (left.is_on_line)
        {
            escapeAngle = 270;
        }
        else
        {
            escapeAngle = 90;
        }

        avoid_start_time = millis();
        current_state = RobotState::AVOIDING_LINE;
        robot.motors.move(escapeAngle, drivePwm);
    }
}