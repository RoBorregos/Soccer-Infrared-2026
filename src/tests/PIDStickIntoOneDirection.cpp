#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;
Bno bno;

#define KP 80/Constants::Motor::maxPWM
#define KI 0/Constants::Motor::maxPWM
#define KD 0/Constants::Motor::maxPWM

#define ERROR_THRESHOLD 100

PID pid(KP, KI, KD, ERROR_THRESHOLD);

double targetYaw = 0.0;


void setup() {
    Serial.begin(9600);
    robot.begin();
    bno.begin();
    delay(2000);
}

void loop() {
    double yaw = bno.GetBNOData();
    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Target Yaw: ");
    double pidOutput = pid.Calculate(targetYaw, yaw);
    Serial.print(" Target Yaw:");
    Serial.print(targetYaw);
    Serial.print(" Yaw: ");
    Serial.println(yaw);
    const float drivePwm = 0.25f * Constants::Motor::maxPWM;
    robot.motors.move(0, drivePwm, pidOutput);

}