#pragma once
#include <Arduino.h>
#include <vector>
#include "binarySerializationData.h"
#include "serializer.h"

class GoodSerializer
{
public:
    void begin(uint32_t serialBaud = 115200, uint32_t uartBaud = 57600);

    // Main loops and endpoints
    void processLoop();
    void testEndpoint();

    // Helpers
    static uint16_t readBE16(uint8_t hi, uint8_t lo);
    bool resyncStream(std::vector<uint8_t>& buffer);
    bool sumTestSerializedData(const std::vector<uint8_t>& packet);

    std::vector<uint8_t> receiveBuffer;

    struct StreamValidationState {
        bool isVerified = false;
        bool hasPrev = false;
        uint16_t prevMag = 0;
        uint16_t prevAng = 0;
        uint32_t checkedTransitions = 0;
    };

    StreamValidationState streamState;
};

#if defined(__AVR_ATmega2560__)
#define UART_PORT Serial1
#else
#include <SoftwareSerial.h>
extern SoftwareSerial UART_PORT;
#endif
