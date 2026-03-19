#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include "constants.h"
#include "moveifball.h"

MoveIfBall moveIfBall(Serial2, Serial);

namespace {
constexpr unsigned long kDebugBaudRate = 9600;
constexpr unsigned long kAuxiliaryBaudRate = 9600;
constexpr unsigned long kPollDelayMs = 100;
}

void setup() {
    Serial.begin(kDebugBaudRate);
    moveIfBall.begin(kAuxiliaryBaudRate);
    Serial.println("Setup complete");
    delay(1000);
}

void loop() {
    if (moveIfBall.ballDetected()) {
        Serial.println("Ball detected!");
    }

    delay(kPollDelayMs);
}