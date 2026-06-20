#ifndef BNO_H
#define BNO_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class Bno {
public:
    Bno();
    void begin();
    double GetBNOData();

private:
    static constexpr uint8_t kPrimaryAddress = 0x4A;
    static constexpr uint8_t kSecondaryAddress = 0x4B;
    static constexpr uint32_t kReportIntervalUs = 10000;
    static constexpr double kRadiansToDegrees = 57.29577951308232;

    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;
    double yaw;
    bool initialized;

    bool beginAtAddress(uint8_t address);
    void enableReports();
    static double normalizeYawDegrees(double angle);
    static double quaternionToYawDegrees(float real, float i, float j, float k);
};

#endif
