/**
 * +-----------+-----+-----+
 * +---------->|   Start   |<-----------------------+
 * |           +-----+-----+                        |
 * |                 |                              |
 * |                 v                              |
 * |               /   \           +----------------+---+
 * |              / Looking \  No  |                    |
 * |             <  forward? >---->|   Orientation PID  |
 * |              \         /      |                    |
 * |               \   /           +--------------------+
 * |                 | Yes                          ^
 * |                 v                              |
 * |               /   \           +----------------+---+
 * |              /  Am I   \  Yes |                    |
 * +-------------<  on line? >---->|    Mirror Move     |
 * |              \         /      |                    |
 * |               \   /           +--------------------+
 * |                 | No
 * |                 v
 * |               /   \             /   \           +-------------------+
 * |              /   Do    \ Yes   / Am I  \  No    |                   |
 * |             <   I have  >---->/in range \------>|    Move Forward   |
 * |              \ possession/    \ to shoot?/      |                   |
 * |               \   /            \   /            +---------+---------+
 * |                 | No             | Yes                    |
 * |                 v                v                        |
 * |          +------------+      +-------+                    |
 * |          |   Search   |      | Shoot |                    |
 * |          |  for Ball  |      +---+---+                    |
 * |          +-----+------+          |                        |
 * |                |                 |                        |
 * +----------------+-----------------+------------------------+
 */

#include <Arduino.h>
#include "constants.h"
#include "robot.h"
#include "HeadingPD.h"
#include "photo.h"
#include "Pixy.h"

Robot robot;
Pixy2 pixy;

/**
 * TEMP CONSTANTS & DECLARATIONS
 * [TODO] - Place this in constants.h 
 * */ 
double targetYaw = 0.0;
const float kHeadingKp = 2.3f;
const float kHeadingKd = 0.18f;
const float kMaxTurnPwm = 90.0f;
const float kMinTurnPwm = 24.0f;
const float kHeadingSettleBandDeg = 2.0f;
const unsigned long kDebugIntervalMs = 100;

static unsigned long lastPixyTime = 0;
static double ballAngle = 0.0;
uint16_t xBallComponent = 0;
uint16_t yBallComponent = 0;

HeadingPD headingPD(kHeadingKp, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);
const float drivePwm = 0.40f * Constants::Motor::maxPWM;
double yaw = 0.0;
double turnCommand = 0.0;

/**
 * TEMP PHOTOS DECLARATION
 * [TODO] - Place this in robot.h & robot.cpp 
 * */ 
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);


enum class RobotState { IDLE, AVOIDING_LINE };
RobotState current_state = RobotState::IDLE;
unsigned long avoid_start_time = 0;
int escapeAngle = 0;

const unsigned long kAvoidDurationMs = 350;
const uint8_t kBaselineSamples = 20;
const uint16_t kBaselineDelayMs = 10;

// Ditto pass to constants.h
#define BALL_SIG      1
#define YELLOW_GOAL   2
#define BLUE_GOAL     3
uint8_t ballSigMap = (uint8_t)(1u << (BALL_SIG - 1));
uint8_t yellowSigMap = (uint8_t)(1u << (YELLOW_GOAL - 1));
uint8_t blueSigMap = (uint8_t)(1u << (BLUE_GOAL - 1));


void setup() {
    Serial.begin(115200);
    robot.begin();
    delay(2000);
    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetMargins(Side::Left , Constants::kLeftMargins, Constants::kPhotoLeftElements);
    phototransistor_sensors.SetMargins(Side::Right, Constants::kRightMargins, Constants::kPhotoRightElements);
    phototransistor_sensors.SetMargins(Side::Front, Constants::kFrontMargins, Constants::kPhotoFrontElements);
    phototransistor_sensors.SetRequiredConfirmations(1);
    phototransistor_sensors.SetThresholdPadding(5);
    phototransistor_sensors.CaptureBaseline(kBaselineSamples, kBaselineDelayMs);

    phototransistor_sensors.GetBaselineReading(Side::Left, 0);
    phototransistor_sensors.GetBaselineReading(Side::Right, 0);
    phototransistor_sensors.GetBaselineReading(Side::Front, 0);

    targetYaw = robot.imu.getAngle();
}

void loop() {
    yaw = robot.imu.getAngle();

    if (millis() - lastPixyTime >= 100) {
        lastPixyTime = millis();

        // Handle BALL_SIG by requesting only that signature (no loop)
        robot.pixy.ccc.getBlocks(true, ballSigMap);
        if (robot.pixy.ccc.numBlocks > 0) {
            Block &blk = robot.pixy.ccc.blocks[0];
            xBallComponent = blk.m_x;
            yBallComponent = blk.m_y;
            double angleDeg = (atan2((double)yBallComponent, (double)xBallComponent) * 180.0 / PI) - 45.0;
            ballAngle = angleDeg;
        }

        robot.pixy.ccc.getBlocks(true, yellowSigMap);
        if (robot.pixy.ccc.numBlocks > 0) {
            // TODO: process yellow goal block: robot.pixy.ccc.blocks[0]
        }

        robot.pixy.ccc.getBlocks(true, blueSigMap);
        if (robot.pixy.ccc.numBlocks > 0) {
            // TODO: process blue goal block: robot.pixy.ccc.blocks[0]
        }
    }

    phototransistor_sensors.CheckPhotosOnField(Side::Left);
    phototransistor_sensors.CheckPhotosOnField(Side::Right);
    phototransistor_sensors.CheckPhotosOnField(Side::Front);

    if (current_state == RobotState::AVOIDING_LINE) {
        if (millis() - avoid_start_time >= kAvoidDurationMs) {
            current_state = RobotState::IDLE;
        } else {
            robot.motors.move(escapeAngle, drivePwm, 0);
            return;
        }
    }

    turnCommand = headingPD.Update(targetYaw, yaw);
    robot.motors.move(ballAngle, drivePwm, turnCommand);
}