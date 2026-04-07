#include <Arduino.h>
#include "photo.h"
#include "constants.h"
#include "PID.h"
#include "robot.h"

Robot robot;
Motors motors;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

double yaw = 0.0;
double targetYaw = 0.0;

const float kDrivePwm = 0.45f * Constants::Motor::maxPWM;
const float kDriveAngle = 0.0f;
const unsigned long kAvoidDurationMs = 350;
const unsigned long kDebugIntervalMs = 200;
const uint8_t kBaselineSamples = 20;
const uint16_t kBaselineDelayMs = 10;

#define KP 400/Constants::Motor::maxPWM
#define KI 150/Constants::Motor::maxPWM
#define KD 50/Constants::Motor::maxPWM


const uint16_t kLeftMargins[Constants::kPhotoLeftElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kRightMargins[Constants::kPhotoRightElements] = {
    50, 50, 50, 50, 50, 50, 50, 50
};
const uint16_t kFrontMargins[Constants::kPhotoFrontElements] = {
    40, 40, 40, 40, 40, 40
};

unsigned long last_debug_time = 0;
unsigned long avoid_start_time = 0;
int escapeAngle = 0;

enum class RobotState { DRIVING_STRAIGHT, AVOIDING_LINE };
RobotState current_state = RobotState::DRIVING_STRAIGHT;

void printSideData(const char *label, Side side, uint8_t num_elements)
{
    Serial.print(label);
    Serial.print(F(": "));
    for (uint8_t channel = 0; channel < num_elements; channel++)
    {
        Serial.print(phototransistor_sensors.GetRawReading(side, channel));
        if (channel < num_elements - 1)
        {
            Serial.print(F("|"));
        }
    }

    Serial.print(F("  thr "));
    Serial.print(label);
    Serial.print(F(": "));
    for (uint8_t channel = 0; channel < num_elements; channel++)
    {
        Serial.print(phototransistor_sensors.GetThreshold(side, channel));
        if (channel < num_elements - 1)
        {
            Serial.print(F("|"));
        }
    }
}

void printRawReadings()
{
    phototransistor_sensors.ReadAllSensors(Side::Left);
    phototransistor_sensors.ReadAllSensors(Side::Right);
    phototransistor_sensors.ReadAllSensors(Side::Front);

    printSideData("L", Side::Left, Constants::kPhotoLeftElements);
    Serial.print(F("  "));
    printSideData("R", Side::Right, Constants::kPhotoRightElements);
    Serial.print(F("  "));
    printSideData("F", Side::Front, Constants::kPhotoFrontElements);
    Serial.println();
}

void setup()
{
    robot.motors.begin();
    robot.motors.stop();
    delay(2000);

    Serial.begin(115200);
    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetMargins(Side::Left, kLeftMargins, Constants::kPhotoLeftElements);
    phototransistor_sensors.SetMargins(Side::Right, kRightMargins, Constants::kPhotoRightElements);
    phototransistor_sensors.SetMargins(Side::Front, kFrontMargins, Constants::kPhotoFrontElements);
    phototransistor_sensors.SetThresholdPadding(5);
    phototransistor_sensors.CaptureBaseline(kBaselineSamples, kBaselineDelayMs);
}

void loop()
{
    yaw = robot.imu.getAngle();

    if (current_state == RobotState::AVOIDING_LINE)
    {
        if (millis() - avoid_start_time >= kAvoidDurationMs)
        {
            current_state = RobotState::DRIVING_STRAIGHT;
        }
        else
        {
            robot.motors.move(escapeAngle, kDrivePwm);
        }
        return;
    }

    if (millis() - last_debug_time >= kDebugIntervalMs)
    {
        printRawReadings();
        last_debug_time = millis();
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

        Serial.print(F("Line detected, escaping at "));
        Serial.println(escapeAngle);

        robot.motors.move(escapeAngle, kDrivePwm);
        return;
    }

    robot.motors.move(kDriveAngle, kDrivePwm);
}
