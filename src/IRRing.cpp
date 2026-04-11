#include "sensor_control.h"
#include "binarySerializationData.h"
#include "serializer.h"
#include "BinaryStreamProcessor.h"
#include <Arduino.h>

/**
 * # Ciclo de la pelota
 * $ \displaystyle \frac{\text{Número de sensores}}{10,000\ \mathrm{ms}} = \text{tiempo de ciclo} $
 * */ 
#define T_MODEA 714
unsigned long time_ms = 0;

float           pulseWidth[IR_NUM]; 
sensorInfo_t    sensorInfo;         
vectorXY_t      vectorXY;          
vectorRT_t      vectorRT;          
vectorRT_t      vectorRTWithSma;  

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

void setup() {
    UART_PORT.begin(kUartBaud);
    setAllSensorPinsInput();
    delay(2000);            // Allow ready time for the receiving Arduino
    lastSendMs = millis();  // Corrected to millis() to match kSendIntervalMs

}

void loop() {
    const uint32_t now = millis();
    if (now - lastSendMs < kSendIntervalMs) {
        return;
    }

    if (!initialTestComplete) {
        // Send predictable +1/+1 verification sequence
        data.ballMagnitude = magnitude++;
        data.ballAngle = angle++;
        
        // Send 15 packets to guarantee receiver catches 5 consecutive valid ones
        if (++testPacketsSent >= 15) {
            initialTestComplete = true;
        }
    } else {
        data.ballMagnitude = sensorInfo.avgPulseWidth; 
        data.ballAngle =  static_cast<uint16_t>(vectorRT.theta);
    }

    serialized = Serializer::serialize(data);
    UART_PORT.write(serialized.data(), serialized.size());
    lastSendMs = now;
}