#include "robot.h"

Robot::Robot() : imu(&IMU, IMU_ADDRESS) {
    currentTime = millis();
}

void Robot::begin() {
    motors.begin();
    // bno.begin();
    int err = imu.begin(calib);
    if (err != 0) {
        Serial.print("IMU begin failed: ");
        Serial.println(err);
    } else {
        Serial.println("IMU initialized");
    }
    irring.init(&currentTime);
}
