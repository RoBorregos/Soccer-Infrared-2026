#include <Arduino.h>
#include "binarySerializationData.h"
#include "serializer.h"
#include "goodSerializer.h"

uint16_t magnitude = 500; 
uint16_t angle = 180;     
IRSerializationData data;
std::vector<uint8_t> serialized;

namespace {
    constexpr uint32_t kUartBaud = 57600;
    constexpr uint32_t kSendIntervalMs = 1;
}

uint32_t lastSendMs = 0;
bool initialTestComplete = false;
uint8_t testPacketsSent = 0;

void setup(){
    UART_PORT.begin(kUartBaud);
    delay(2000);            // Allow ready time for the receiving Arduino
    lastSendMs = millis();  // Corrected to millis() to match kSendIntervalMs
}

void loop(){
    const uint32_t now = millis(); // Corrected to millis()
    if (now - lastSendMs < kSendIntervalMs) {
        return;
    }

    if (!initialTestComplete) {
        // Phase 1: Send predictable +1/+1 verification sequence
        data.ballMagnitude = magnitude++;
        data.ballAngle = angle++;
        
        // Send 15 packets to guarantee receiver catches 5 consecutive valid ones
        if (++testPacketsSent >= 15) {
            initialTestComplete = true;
        }
    } else {
        // Phase 2: Simulate real, unpredictable sensor data
        data.ballMagnitude = data.ballMagnitude + random(-4, 5); 
        data.ballAngle = data.ballAngle + random(-1, 2);
    }

    serialized = Serializer::serialize(data);
    UART_PORT.write(serialized.data(), serialized.size());

    lastSendMs = now;
}