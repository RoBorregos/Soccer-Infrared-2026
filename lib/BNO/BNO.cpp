#include "BNO.h"
#include <math.h>

Bno::Bno() : bno(-1), sensorValue{}, yaw(0.0), initialized(false) {
}

void Bno::begin() {
    initialized = beginAtAddress(kPrimaryAddress) || beginAtAddress(kSecondaryAddress);
    if (!initialized) {
        return;
    }

    enableReports();
    delay(100);
}

double Bno::GetBNOData() {
    if (!initialized) {
        return yaw;
    }

    if (bno.wasReset()) {
        enableReports();
    }

    if (!bno.getSensorEvent(&sensorValue)) {
        return yaw;
    }

    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        const sh2_RotationVector_t &rotation = sensorValue.un.gameRotationVector;
        yaw = quaternionToYawDegrees(rotation.real, rotation.i, rotation.j, rotation.k);
        yaw = normalizeYawDegrees(yaw);
        yaw = -yaw; // Change sign to match the motors motion function
    }

    return yaw;
}

bool Bno::beginAtAddress(uint8_t address) {
    if (!bno.begin_I2C(address, &Wire)) {
        return false;
    }

    (void)address;
    return true;
}

void Bno::enableReports() {
    // GAME_ROTATION_VECTOR is the closest BNO085 equivalent to the old IMUPLUS mode.
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, kReportIntervalUs);
}

double Bno::normalizeYawDegrees(double angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

double Bno::quaternionToYawDegrees(float real, float i, float j, float k) {
    const double sqr = static_cast<double>(real) * real;
    const double sqi = static_cast<double>(i) * i;
    const double sqj = static_cast<double>(j) * j;
    const double sqk = static_cast<double>(k) * k;
    const double numerator = 2.0 * (static_cast<double>(i) * j + static_cast<double>(k) * real);
    const double denominator = sqi - sqj - sqk + sqr;

    return atan2(numerator, denominator) * kRadiansToDegrees;
}
