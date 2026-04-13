#include "robot.h"

Robot::Robot() : imu(&IMU, IMU_ADDRESS) {
    currentTime = millis();
}

void Robot::begin() {
    // Motors
    motors.begin();

    // BNO055
    // bno.begin();

    // IMU
    Wire.begin();
    Wire.setClock(400000);
    int err = imu.begin(calib);
    if (err != 0) {
        Serial.print("IMU begin failed: ");
        Serial.println(err);
    } else {
        Serial.println("IMU initialized");
    }

    // IR Ring
    irring.init(&currentTime);
}
