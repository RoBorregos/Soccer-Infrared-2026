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

const float drivePwm = 0.75f * Constants::Motor::maxPWM;
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
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);
}

void loop()
{
    if (current_state == RobotState::AVOIDING_LINE)
    {
        if (millis() - avoid_start_time >= Constants::kAvoidDurationMs)
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
    
    int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
    if (escapeAngleDetected != -1)
    {
        escapeAngle = escapeAngleDetected;
        avoid_start_time = millis();
        current_state = RobotState::AVOIDING_LINE;
        robot.motors.move(escapeAngle, drivePwm);
    }
}