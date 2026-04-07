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

    // Pixy2
    int result = pixy.init();
    if (result == 0) {
        Serial.println("[SUCCESS] Pixy2 connected!");
    } else {
        Serial.println("[FAIL] Pixy2 not found. Check wiring, pixyMon interface configs, energy supply or whatever idk lol");
        while (true); 
    }
}
