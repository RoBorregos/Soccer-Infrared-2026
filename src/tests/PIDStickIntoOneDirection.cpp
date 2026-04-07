#include <Arduino.h>
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;

#define KP 600/Constants::Motor::maxPWM
#define KI 250/Constants::Motor::maxPWM
#define KD 50/Constants::Motor::maxPWM

#define ERROR_THRESHOLD 100

PID pid(KP, KI, KD, ERROR_THRESHOLD);

double targetYaw = 0.0;

void setup() {
    Serial.begin(115200);
    robot.begin();
    delay(2000);
}

void loop() {
    double yaw = robot.imu.getAngle();
    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Target Yaw: ");
    double pidOutput = pid.Calculate(targetYaw, yaw);
    Serial.print(" Target Yaw:");
    Serial.print(targetYaw);
    Serial.print(" Yaw: ");
    Serial.println(yaw);
    const float drivePwm = 0.60f * Constants::Motor::maxPWM;
    robot.motors.move(0, drivePwm, pidOutput);

}