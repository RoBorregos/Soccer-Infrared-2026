#include "IRRing.h"
#include <Arduino.h>
#include <SingleEMAFilterLib.h>
#include "constants.h"

SingleEMAFilter<double> filterAngle(0.6);
SingleEMAFilter<double> filterStr(0.6);
#define BALLANGLECORRECTION 0.0

IRRing::IRRing() :
    angle(0),
    strength(0),
    offset(0),
    current_time(nullptr),
    last_time(0) {
}

void IRRing::init(unsigned long* current_time) {
    this->current_time = current_time;
    Serial1.begin(Constants::kIRSerialBaud);
    Serial1.setTimeout(100);
}

void IRRing::UpdateData() {
    while (Serial1.available()) {
        String data = Serial1.readStringUntil('\n');
        data.trim();

        if (data.length() == 0) {
            continue;
        }

        if (data.length() > 2 && data[1] == ' ') {
            const char type = data[0];
            const String valueStr = data.substring(2);
            if (valueStr.length() == 0) {
                continue;
            }

            const double value = valueStr.toFloat();
            if (type == 'a') {
                angle = value + offset;
                filterAngle.AddValue(angle);
                hasAngleReading = true;
                last_time = millis();
            } else if (type == 'r') {
                strength = value;
                filterStr.AddValue(strength);
            }
            continue;
        }

        angle = data.toFloat() + offset;
        filterAngle.AddValue(angle);
        hasAngleReading = true;
        last_time = millis();
    }
}

void IRRing::SetOffset(double offset) {
    this->offset = offset;
}

double IRRing::GetRawAngle() {
    double currentAngle = hasAngleReading ? filterAngle.GetLowPass() : angle;
    if (currentAngle > 180) {
        currentAngle -= 360;
    }
    return currentAngle;
}

double IRRing::GetStrength() {
    return filterStr.GetLowPass();
}

bool IRRing::HasFreshData(unsigned long timeoutMs) const {
    return hasAngleReading && (millis() - last_time <= timeoutMs);
}

double IRRing::GetAngle(float ballFollowOffsetBack, float ballFollowOffsetSide, float ballFollowOffsetFront) {
    double currentAngle = hasAngleReading ? filterAngle.GetLowPass() : angle;
    currentAngle = fmod(currentAngle + 180.0, 360.0);
    if (currentAngle < 0) currentAngle += 360.0;
    currentAngle -= 180.0;

    float magnitude = abs(currentAngle);
    if (magnitude > 52) {
        currentAngle *= ballFollowOffsetBack;
    } else if (magnitude > 25) {
        currentAngle *= ballFollowOffsetSide;
    } else {
        currentAngle *= ballFollowOffsetFront;
    }

    if (magnitude > 0.05) {
        lastBallAngle = currentAngle;
    } else {
        currentAngle = lastBallAngle;
    }

    double finalResult = (currentAngle * -1) + BALLANGLECORRECTION;
    finalResult = fmod(finalResult + 180.0, 360.0);
    if (finalResult < 0) finalResult += 360.0;

    return finalResult - 180.0;
}
