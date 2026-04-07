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
#include "PID.h"
#include "photo.h"
#include "Pixy.h"

Robot robot;
Pixy2 pixy;
/**
 * TEMP CONSTANTS & DECLARATIONS
 * [TODO] - Place this in constants.h 
 * */ 
double targetYaw = 0.0;
const double kHeadingKp = 2.3f;
const double kHeadingKd = 0.18f;
const double kMaxTurnPwm = 90.0f;
const double kMinTurnPwm = 24.0f;
const double kHeadingSettleBandDeg = 2.0f;
const unsigned long kDebugIntervalMs = 100;

static unsigned long lastPixyTime = 0;
static double ballAngle = 0.0;
uint16_t xBallComponent = 0;
uint16_t yBallComponent = 0;

PID headingPD(kHeadingKp, 0.0, kHeadingKd, kMinTurnPwm, kMaxTurnPwm, kHeadingSettleBandDeg);
const double drivePwm = 0.40f * Constants::Motor::maxPWM;
const double escapePwm = 0.60f * Constants::Motor::maxPWM;
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
const uint8_t kBaselineSamples = 25;
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
    pinMode(LED_BUILTIN, OUTPUT);
    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetMargins(Side::Front, Constants::kFrontMargins, Constants::kPhotoFrontElements);
    phototransistor_sensors.SetThresholdPadding(15);
    phototransistor_sensors.CaptureBaseline(kBaselineSamples, kBaselineDelayMs);
    phototransistor_sensors.GetBaselineReading(Side::Left, 0);
    phototransistor_sensors.GetBaselineReading(Side::Right, 0);
    phototransistor_sensors.GetBaselineReading(Side::Front, 0);


    targetYaw = robot.imu.getAngle();
    delay(2000);

    digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {

    /**
     * IMU getters
     */
    yaw = robot.imu.getAngle();

    /**
     * Vision Getters
     */
    if (millis() - lastPixyTime >= 100) {
        lastPixyTime = millis();

        // Handle BALL_SIG
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
            // TODO: process yellow goal block
        }

        robot.pixy.ccc.getBlocks(true, blueSigMap);
        if (robot.pixy.ccc.numBlocks > 0) {
            // TODO: process blue goal block
        }
    }

    /**
     * 1. CRITICAL PRIORITY: Line Detection
     */
    // PhotoData left = phototransistor_sensors.CheckPhotosOnField(Side::Left);
    // PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);
    PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);

    // Only consider front photos for escape angle to test front line reaction
    int detectedEscapeAngle = Phototransistor::GetEscapeAngle(front); 
    
    // Trigger escape if a line is seen and we aren't already escaping
    if (detectedEscapeAngle != -1 && current_state != RobotState::AVOIDING_LINE) {
        escapeAngle = detectedEscapeAngle;
        avoid_start_time = millis();
        current_state = RobotState::AVOIDING_LINE;
    }

    /**
     * 2. CRITICAL PRIORITY: Avoidance Movement
     */
    if (current_state == RobotState::AVOIDING_LINE) {
        if (millis() - avoid_start_time >= kAvoidDurationMs) {
            robot.motors.stop();
            current_state = RobotState::IDLE;
        } else {
            robot.motors.move(escapeAngle, escapePwm);
        }
        
        // EARLY RETURN: Ignore IMU, Pixy, and normal driving entirely while escaping!
        return; 
    }

    /**
     * 3. Default Movement
     */
    turnCommand = headingPD.calculate(targetYaw, yaw);
    robot.motors.move(ballAngle, drivePwm, turnCommand);
}