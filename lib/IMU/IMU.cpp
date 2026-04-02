#include "IMU.h"
IMUDriver::IMUDriver(MPU6500* imuPtr, uint8_t address) :
    imu(imuPtr), addr(address), gyro_bias_z(0.0f), yaw(0.0f), previous_time(0) {}

int IMUDriver::begin(calData &cal, int samples = 500) {
    int err = imu->init(cal, addr);
    if (err != 0) return err;

    // quick calibration of gyro Z
    Serial.println("Calibrating gyro Z bias, DO NOT MOVE...");
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
      imu->update();
      imu->getGyro(&gyroData);
      sum += gyroData.gyroZ;
      delay(5);
    }
    gyro_bias_z = sum / samples;
    Serial.print("Calculated bias: ");
    Serial.println(gyro_bias_z);

    previous_time = micros();
    return 0;
}

// Call regularly to get fused yaw angle (degrees)
/**
 * Mathematical model:
 * From an angular velocity vector (gyroData) we want to c-
 * -ompute an angle (yaw). We do this by integrating the a-
 * -ngular velocity over time. Resulting on the yaw angle 
 * in degrees:
 * 
 * \theta(t) = \int_0^t \omega_z(t) dt
 * 
 * We also apply a deadband to the gyro readings because
 * of the gyro's drift, which causes the yaw to diverge
 * over time. This precautionary measurement makes it so 
 * that this happens at a much slower rate or not at all.
 */
float IMUDriver::getAngle() {
    unsigned long current_time = micros();
    double dt = (current_time - previous_time) / 1000000.0;
    // protect against very large dt on first call
    if (dt <= 0 || dt > Constants::kIMUMaxDt) dt = 0; 

    imu->update();
    imu->getGyro(&gyroData);

    float gyro_z = gyroData.gyroZ - gyro_bias_z;
    if (fabs(gyro_z) < Constants::kIMUDeadbandThreshold) gyro_z = 0.0f; // deadband

    yaw += gyro_z * dt;

    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    previous_time = current_time;
    return yaw;
}