#include <Arduino.h>
#include "photo.h"
#include "constants.h"
#include <math.h>
#include "robot.h"
#include "BNO.h"
#include "IRRing.h"
#include "PID.h"

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);
IRRing irring;
Bno bno;

#define KP (180.0f / Constants::Motor::maxPWM)
#define KI (42.0f / Constants::Motor::maxPWM)
#define KD (50.0f / Constants::Motor::maxPWM)
#define ERROR_THRESHOLD 100

PID pid(KP, KI, KD, ERROR_THRESHOLD);

float kBallFollowOffsetBack = 1.2;
float kBallFollowOffsetSide = 1.0;
float kBallFollowOffsetFront = 1.0;

double targetYaw = 0.0;
double BNOCORRECTION = 0.0;

static double WrapAngle180(double deg) {
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

enum class RobotState { CHASING_BALL, AVOIDING_LINE };
RobotState current_state = RobotState::CHASING_BALL;

// Line avoidance variables
unsigned long avoid_start_time = 0;
const unsigned long kAvoidDurationMs = 300; 
int escapeAngle = 0; 

void setup() {
    Serial.begin(115200);
    phototransistor_sensors.Initialize();
    bno.begin();
    robot.begin();
    
    unsigned long currentTime = millis();
    irring.init(&currentTime);
    irring.SetOffset(0.0);
    
    delay(1000);
    BNOCORRECTION = -bno.GetBNOData();
    Serial.println("Ready!");
}

void loop() {
    const float drivePwm = 0.25f * Constants::Motor::maxPWM; 
    
    irring.UpdateData();
    double yaw = WrapAngle180(bno.GetBNOData() + BNOCORRECTION);
    double pidSpeed = pid.Calculate(targetYaw, yaw);

    if (current_state == RobotState::AVOIDING_LINE) {
        
        if (millis() - avoid_start_time >= kAvoidDurationMs) {
            current_state = RobotState::CHASING_BALL; 
        } 
        else {
            robot.move(escapeAngle, drivePwm, pidSpeed);
            return; 
        }
    }
    if (current_state == RobotState::CHASING_BALL) {
        
        PhotoData left  = phototransistor_sensors.CheckPhotosOnField(Side::Left);
        PhotoData right = phototransistor_sensors.CheckPhotosOnField(Side::Right);
        PhotoData front = phototransistor_sensors.CheckPhotosOnField(Side::Front);

        if (front.is_on_line || left.is_on_line || right.is_on_line) {
            
            if (front.is_on_line) escapeAngle = 180;
            else if (left.is_on_line) escapeAngle = 90;
            else if (right.is_on_line) escapeAngle = -90;

            current_state = RobotState::AVOIDING_LINE;
            avoid_start_time = millis(); 

            Serial.print("-------Line-------");
            Serial.println(escapeAngle);
            
            robot.move(escapeAngle, drivePwm, pidSpeed);
        }
        else {
            double ballAngle = irring.GetAngle(kBallFollowOffsetBack, kBallFollowOffsetSide, kBallFollowOffsetFront);
            
            robot.move(-ballAngle, drivePwm, pidSpeed); 
        }
    }
}