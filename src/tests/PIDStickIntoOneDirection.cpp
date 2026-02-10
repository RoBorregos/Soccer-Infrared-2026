#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;
Bno bno;

#define KP 0.1
#define KI 0.01
#define KD 0.05

#define ERROR_THRESHOLD 5.0

void setup() {
    Serial.begin(9600);
    robot.begin();
    bno.begin();
    delay(2000);
}

void loop() {
    double yaw = bno.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    double targetYaw = 0.0;
    PIDParameters pidParams(KP, KI, KD, Constants::Motor::maxPWM, Constants::Motor::minPWM, ERROR_THRESHOLD);

    pidParams.target = targetYaw;
    pidParams.current_value = yaw;
    double pidOutput = PID::calculate(pidParams);
    robot.move(0, 0.5, pidOutput);
}