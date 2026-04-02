#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "FastIMU.h"
#include "constants.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68

class IMUDriver {
public:
  IMUDriver(MPU6500* imuPtr, uint8_t address);
  // Initialize IMU and compute gyro Z bias (blocking calibration)
  int begin(calData &cal, int samples = 500);
  float getAngle();
private:
  MPU6500* imu;
  uint8_t addr;
  GyroData gyroData;
  float gyro_bias_z;
  float yaw;
  unsigned long previous_time;
};

#endif // IMU_DRIVER_H