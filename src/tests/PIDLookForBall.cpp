#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"
#include "IRRing.h"

IRRing irring;
Robot robot;
Bno bno;

#define KP 180/Constants::Motor::maxPWM
#define KI 42/Constants::Motor::maxPWM
#define KD 50/Constants::Motor::maxPWM

float kBallFollowOffsetBack = 1.2;
float kBallFollowOffsetSide = 1.0;
float kBallFollowOffsetFront = 1.0;

// #define KP 120/Constants::Motor::maxPWM
// #define KI 24/Constants::Motor::maxPWM
// #define KD 24/Constants::Motor::maxPWM

#define ERROR_THRESHOLD 100

// // Persistent PID parameters so I/D terms accumulate across loop() calls
// PIDParameters pidParams(KP, KI, KD,
//                         Constants::Motor::maxPWM,
//                         0,
//                         ERROR_THRESHOLD);

// 0.9375/kMaxPWM, 0.01/kMaxPWM, 0.01/kMaxPWM
PID pid(KP, KI, KD, ERROR_THRESHOLD);

// #define BNOINITIALCORRECTION 136.56
double targetYaw = 0.0;
double BNOCORRECTION = 0.0;

static double WrapAngle180(double deg) {
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

void setup() {
    Serial.begin(115200);
    robot.begin();
    bno.begin();
    delay(1000);

    // Capture initial yaw as an offset so "current" yaw starts near 0.
    double initialYaw = bno.GetBNOData();
    BNOCORRECTION = -initialYaw;

    // robot.begin();
    unsigned long currentTime = millis();
    irring.init(&currentTime);
    irring.SetOffset(0.0);
    irring.UpdateData();
    delay(1000);
    
}

void loop() {
    irring.UpdateData();
    // double yaw = bno.GetBNOData() + BNOINITIALCORRECTION;
    double yaw = WrapAngle180(bno.GetBNOData() + BNOCORRECTION);
    double ballAngle = irring.GetAngle(kBallFollowOffsetBack, kBallFollowOffsetSide, kBallFollowOffsetFront);
    double speed = pid.Calculate(targetYaw, yaw);
    double ballAngleRelativeToRobot = ballAngle - yaw;
    Serial.print(" BOut: ");
    Serial.print(ballAngle);
    Serial.print(" DAB: ");
    Serial.print(ballAngleRelativeToRobot);
    Serial.print(" Yaw: ");
    Serial.println(yaw);

    // NOTE: Motor::setSpeed floors to an integer PWM value.
    // If we pass 0.35f directly, PWM becomes 0 and the robot won't translate.
    const float drivePwm = 0.35f * Constants::Motor::maxPWM;
    robot.move(-ballAngle, drivePwm, speed);
}