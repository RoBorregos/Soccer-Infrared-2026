#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"
#include "IRRing.h"

IRRing irring;
Robot robot;
Bno bno;

#define KP 140/Constants::Motor::maxPWM
#define KI 42/Constants::Motor::maxPWM
#define KD 18/Constants::Motor::maxPWM

#define ERROR_THRESHOLD 100

// // Persistent PID parameters so I/D terms accumulate across loop() calls
// PIDParameters pidParams(KP, KI, KD,
//                         Constants::Motor::maxPWM,
//                         0,
//                         ERROR_THRESHOLD);

// 0.9375/kMaxPWM, 0.01/kMaxPWM, 0.01/kMaxPWM
PID pid(KP, KI, KD, ERROR_THRESHOLD);

#define INITIALANGLECORRECTION 90.0
double targetYaw = {};

void setup() {
    Serial.begin(115200);
    robot.begin();
    bno.begin();
    unsigned long currentTime = millis();
    irring.init(&currentTime);
    irring.SetOffset(0.0);
    delay(2000);
    irring.UpdateData();


}

void loop() {
    irring.UpdateData();
    double yaw = bno.GetBNOData();
    // Serial.print("Yaw: ");
    // Serial.print(yaw);
    targetYaw = INITIALANGLECORRECTION + irring.GetAngle(1.0, 1.0, 1.0);
    // double pidOutput = pid.Calculate(targetYaw, yaw);
    Serial.print(" Target Yaw:");
    Serial.print(targetYaw);
    Serial.print(" Yaw: ");
    Serial.println(yaw);;
    // robot.move(0, 0.35f, pidOutput);
}