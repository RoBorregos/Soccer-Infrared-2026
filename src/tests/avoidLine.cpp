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

const float         drivePwm         = 0.45f * Constants::Motor::maxPWM;
const unsigned long kAvoidDurationMs = 500;
const unsigned long kDebugIntervalMs = 200;

enum class RobotState { IDLE, AVOIDING_LINE };
RobotState current_state = RobotState::IDLE;

unsigned long avoid_start_time = 0;
unsigned long last_debug_time  = 0;
int escapeAngle = 0;

void printRawReadings() {
    phototransistor_sensors.ReadAllSensors(Side::Left);
    phototransistor_sensors.ReadAllSensors(Side::Right);
    phototransistor_sensors.ReadAllSensors(Side::Front);

    Serial.print(F("L: "));
    for (int i = 0; i < Constants::kPhotoLeftElements; i++) {
        Serial.print(phototransistor_sensors.GetRawReading(Side::Left, i));
        if (i < Constants::kPhotoLeftElements - 1) Serial.print(F("|"));
    }

    Serial.print(F("  R: "));
    for (int i = 0; i < Constants::kPhotoRightElements; i++) {
        Serial.print(phototransistor_sensors.GetRawReading(Side::Right, i));
        if (i < Constants::kPhotoRightElements - 1) Serial.print(F("|"));
    }

    Serial.print(F("  F: "));
    for (int i = 0; i < Constants::kPhotoFrontElements; i++) {
        Serial.print(phototransistor_sensors.GetRawReading(Side::Front, i));
        if (i < Constants::kPhotoFrontElements - 1) Serial.print(F("|"));
    }

    Serial.print(F("  thr L/R/F: "));
    Serial.print(Constants::kPhotoTresholdLeft);
    Serial.print(F("/"));
    Serial.print(Constants::kPhotoTresholdRight);
    Serial.print(F("/"));
    Serial.println(Constants::kPhotoTresholdFront);
}

void setup() {
    robot.begin();
    robot.stop(); 
    delay(2000);  
    
    Serial.begin(115200);
    phototransistor_sensors.Initialize();
}

void loop() {
    if (current_state == RobotState::AVOIDING_LINE) {
        if (millis() - avoid_start_time >= kAvoidDurationMs) {
            robot.stop();
            current_state = RobotState::IDLE;
        } else {
            robot.move(escapeAngle, drivePwm);
        }
        return;
    }

    // Debug print while IDLE
    if (millis() - last_debug_time >= kDebugIntervalMs) {
        printRawReadings();
        last_debug_time = millis();
    }

    // IDLE — check for line
    PhotoData left  = phototransistor_sensors.CheckPhotosOnField(Side::Left);
    PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);
    PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);

    if (front.is_on_line || left.is_on_line || right.is_on_line) {
        if      (front.is_on_line) escapeAngle = 180;
        else if (left.is_on_line)  escapeAngle =  270;
        else                       escapeAngle =  90;

        avoid_start_time = millis();
        current_state    = RobotState::AVOIDING_LINE;

        Serial.print(F("Line detected — escaping at "));
        Serial.println(escapeAngle); 

        robot.move(escapeAngle, drivePwm);
    }
}