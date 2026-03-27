#include <Arduino.h>
#include "photo.h"
#include "constants.h"
#include "robot.h"
#include "BNO.h"
#include "PID.h"
#include "pixy2.h"
#include <math.h>

// ============================================================
//  PixyLineAvoidance.cpp
//
//  Follows the ball using Pixy2 camera with PID yaw correction.
//  When a line is detected, escapes then resumes chasing.
//  When no ball is visible, stops and waits.
// ============================================================

Pixy2 pixy;
Robot robot;
Bno bno;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

#define KP (160.0f / Constants::Motor::maxPWM)
#define KI ( 36.0f / Constants::Motor::maxPWM)
#define KD ( 50.0f / Constants::Motor::maxPWM)
#define ERROR_THRESHOLD 100

PID pid(KP, KI, KD, ERROR_THRESHOLD);

const float         drivePwm         = 0.45f * Constants::Motor::maxPWM;
const unsigned long kAvoidDurationMs = 500;

double targetYaw  = 0.0;
double ballAngle  = 0.0;
bool   ballVisible = false;

enum class RobotState { CHASING_BALL, AVOIDING_LINE };
RobotState current_state = RobotState::CHASING_BALL;

unsigned long avoid_start_time = 0;
int escapeAngle = 0;

static double WrapAngle180(double deg) {
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

void setup() {
    robot.begin();
    robot.stop();
    delay(2000);

    Serial.begin(115200);
    phototransistor_sensors.Initialize();
    bno.begin();
    delay(1000);

    int result = pixy.init();
    if (result == 0) {
        Serial.println(F("[SUCCESS] Pixy2 connected!"));
    } else {
        Serial.println(F("[FAIL] Pixy2 not found. Check wiring or power."));
        while (true);
    }

    delay(1000);
}

void loop() {
    double yaw      = WrapAngle180(bno.GetBNOData());
    double pidSpeed = pid.Calculate(targetYaw, yaw);

    // --- Line avoidance state ---
    if (current_state == RobotState::AVOIDING_LINE) {
        if (millis() - avoid_start_time >= kAvoidDurationMs) {
            current_state = RobotState::CHASING_BALL;
        } else {
            robot.move(escapeAngle, drivePwm);
        }
        return;
    }

    // --- Update Pixy every 100ms ---
    static unsigned long lastPixyTime = 0;
    if (millis() - lastPixyTime >= 100) {
        lastPixyTime = millis();
        pixy.ccc.getBlocks();
        ballVisible = false;

        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == 1) {
                ballVisible = true;
                uint16_t xComponent = pixy.ccc.blocks[i].m_x;
                uint16_t yComponent = pixy.ccc.blocks[i].m_y;
                ballAngle = (atan2((double)yComponent, (double)xComponent) * 180.0 / PI) - 45.0;

                Serial.print(F("BALL| x= ")); Serial.print(xComponent);
                Serial.print(F("  y= "));    Serial.print(yComponent);
                Serial.print(F("  angle= ")); Serial.println(ballAngle);
                break;
            }
        }

        if (!ballVisible) Serial.println(F("No ball detected"));
    }

    // --- Check for line ---
    PhotoData left  = phototransistor_sensors.CheckPhotosOnField(Side::Left);
    PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);
    PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);

    if (front.is_on_line || left.is_on_line || right.is_on_line) {
        if      (front.is_on_line) escapeAngle = 180;
        else if (left.is_on_line)  escapeAngle = 270;
        else                       escapeAngle =  90;

        avoid_start_time = millis();
        current_state    = RobotState::AVOIDING_LINE;

        Serial.print(F("Line detected — escaping at "));
        Serial.println(escapeAngle);

        robot.move(escapeAngle, drivePwm);
        return;
    }

    // --- Chase ball or stop ---
    if (ballVisible) {
        double ballAngleRelativeToRobot = ballAngle + yaw;
        robot.move(ballAngle, drivePwm, pidSpeed);
    } else {
        robot.stop();
    }
}