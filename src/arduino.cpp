#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"

Robot robot;
Bno bno;

#define KP 0.0
#define KI 0
#define KD 0

#define ERROR_THRESHOLD 5

void setup() {
    Serial.begin(9600);
    robot.begin();
    bno.begin();
    Serial.println("Setup complete");
    delay(2000);
}

void loop() {
    double yaw = bno.GetBNOData();
    double targetYaw = 0.0;
    PIDParameters pidParams(KP, KI, KD, Constants::Motor::maxPWM, 0, ERROR_THRESHOLD);

    pidParams.target = targetYaw;
    pidParams.current_value = yaw;
    double pidOutput = PID::calculate(pidParams);
    Serial.print(pidOutput);
    robot.move(0, 0.0, pidOutput);
}