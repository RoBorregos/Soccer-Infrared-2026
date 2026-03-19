#pragma once
#include <Arduino.h>
#include "binarySerializationData.h"
#include "serializer.h"


class GoodSerializer
{
public:
    void begin(uint32_t serialBaud = 115200, uint32_t uartBaud = 57600);
    void processLoop();

    static uint16_t readBE16(uint8_t hi, uint8_t lo);
    bool testSerializedData(const std::vector<uint8_t>& packet);
    bool resyncStream(std::vector<uint8_t>& buffer);
    bool sumTestSerializedData(const std::vector<uint8_t>& packet, uint16_t expectedMag, uint16_t expectedAng);

    std::vector<uint8_t> receiveBuffer;

    struct StreamValidationState {
        bool hasPrev = false;
        bool passPrinted = false;
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