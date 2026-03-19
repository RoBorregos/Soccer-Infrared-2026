#include <Arduino.h>
#include "binarySerializationData.h"
#include "serializer.h"
#include "goodSerializer.h"

uint16_t magnitude = 0; // Starting at 101 (0x0065)
uint16_t angle = 2;     // Starting at 106 (0x006A)
IRSerializationData data;
std::vector<uint8_t> serialized = Serializer::serialize(data);

namespace {
constexpr uint32_t kUartBaud = 57600;
constexpr uint32_t kSendIntervalMs = 100;
}

uint32_t sentCount = 0;
uint32_t lastSendMs = 0;

void setup(){
    UART_PORT.begin(kUartBaud);
    delay(2000);            // Allow ready time for the receiving Arduino
    lastSendMs = micros();
}

void loop(){
    const uint32_t now = micros();
    if (now - lastSendMs < kSendIntervalMs) {
        return;
    }

    data.ballMagnitude = magnitude;
    data.ballAngle = angle;
    serialized = Serializer::serialize(data);
    UART_PORT.write(serialized.data(), serialized.size());

    ++magnitude;
    ++angle;
    ++sentCount;
    lastSendMs = now;
}