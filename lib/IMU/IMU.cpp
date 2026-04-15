#include "IMU.h"
#include "dependencies/IMUUtils.cpp"
#include "dependencies/F_MPU6500.cpp"

// Define globals in one translation unit to avoid multiple-definition
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;

IMUDriver::IMUDriver(MPU6500* imuPtr, uint8_t address) :
    imu(imuPtr), addr(address), gyro_bias_z(0.0f), yaw(0.0f), previous_time(0) {}

int IMUDriver::begin(calData &cal, int samples) {
    int err = imu->init(cal, addr);
    if (err != 0) return err;

    // quick calibration of gyro Z
    Serial.println("Calibrando Giroscopio... NO MOVER");
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
      imu->update();
      imu->getGyro(&gyroData);
      sum += gyroData.gyroZ;
      delay(5);
    }
    gyro_bias_z = sum / samples;
    Serial.print("Bias calculado: ");
    Serial.println(gyro_bias_z);

    previous_time = micros();
    return 0;
}

// Call regularly to get fused yaw angle (degrees)
float IMUDriver::getAngle() {
    unsigned long current_time = micros();
    double dt = (current_time - previous_time) / 1000000.0;
    // protect against very large dt on first call
    if (dt <= 0 || dt > 0.1) dt = 0; 

    imu->update();
    imu->getGyro(&gyroData);

    float gyro_z = gyroData.gyroZ - gyro_bias_z;
    if (abs(gyro_z) < 0.05f) gyro_z = 0.0f; // deadband

    yaw += gyro_z * dt;

    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    previous_time = current_time;
    return yaw;
}
