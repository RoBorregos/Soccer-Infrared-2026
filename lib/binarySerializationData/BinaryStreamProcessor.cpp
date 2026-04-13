#include "BinaryStreamProcessor.h"
#define MAX_ATTEMPTS 5
#define RX 0
#define TX 1

#if !defined(CORE_TEENSY) && !defined(TEENSYDUINO) && !defined(__AVR_ATmega2560__)
#include <SoftwareSerial.h>
SoftwareSerial UART_PORT(RX, TX); // For main Arduino.
#endif

void BinaryStreamProcessor::begin(uint32_t serialBaud, uint32_t uartBaud) {
    Serial.begin(serialBaud);
    UART_PORT.begin(uartBaud);
    Serial.println("Main arduino is ready and listening...");
    Serial.println("Awaiting initial sum test sequence...");
}

// Dedicated endpoint for the initial verification phase
void BinaryStreamProcessor::testEndpoint() {
    while (UART_PORT.available() > 0) {
        receiveBuffer.push_back(static_cast<uint8_t>(UART_PORT.read()));
    }

    if (receiveBuffer.size() < Serializer::RECORD_SIZE) return;

    std::vector<uint8_t> packet(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);

    if (!sumTestSerializedData(packet)) {
        // If the sum test fails, fallback to your codebase's bitshift method
        if (!resyncStream(receiveBuffer)) {
            // Keep buffer bounded if resync completely fails
            const size_t maxKeep = Serializer::RECORD_SIZE * 4;
            if (receiveBuffer.size() > maxKeep) {
                receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + (receiveBuffer.size() - maxKeep));
            }
            return;
        }
    } else {
        // Valid test packet -> consume it
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);
    }
}

void BinaryStreamProcessor::processLoop() {
    // Phase 1: Run the initial test until verified
    if (!streamState.isVerified) {
        testEndpoint();
        return;
    }

    // Phase 2: Sensor Data (Non-blocking, no sum testing)
    while (UART_PORT.available() > 0) {
        receiveBuffer.push_back(static_cast<uint8_t>(UART_PORT.read()));
    }

    if (receiveBuffer.size() < Serializer::RECORD_SIZE) return;

    std::vector<uint8_t> packet(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);
    
    // Consume the packet
    receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);

    std::vector<IRSerializationData> deserialized = Serializer::deserialize(packet);
    if (deserialized.empty()) {
        Serial.println("Deserialization failed or empty packet");
        return;
    }

    // Output real sensor data
    Serial.print("SENSOR M=");
    Serial.print(deserialized[0].ballMagnitude);
    Serial.print(" ang=");
    Serial.println(deserialized[0].ballAngle);
}

/**
 * Grabs the first four bytes as little endians and returns the combined uint16_ts. 
 * This is used for both magnitude and angle extraction.
 */
uint16_t BinaryStreamProcessor::readBE16(uint8_t hi, uint8_t lo) {
    return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

// Refactored to act as the primary sequence checker
bool BinaryStreamProcessor::sumTestSerializedData(const std::vector<uint8_t>& packet) {
    if (packet.size() < Serializer::RECORD_SIZE) return false;
    
    uint16_t mag = readBE16(packet[0], packet[1]);
    uint16_t ang = readBE16(packet[2], packet[3]);

    if (!streamState.hasPrev) {
        streamState.hasPrev = true;
        streamState.prevMag = mag;
        streamState.prevAng = ang;
        return true; // Accept first packet as baseline
    }

    const uint16_t expectedMag = static_cast<uint16_t>(streamState.prevMag + 1);
    const uint16_t expectedAng = static_cast<uint16_t>(streamState.prevAng + 1);

    if (mag == expectedMag && ang == expectedAng) {
        streamState.prevMag = mag;
        streamState.prevAng = ang;
        ++streamState.checkedTransitions;

        if (!streamState.isVerified && streamState.checkedTransitions >= MAX_ATTEMPTS) {
            streamState.isVerified = true; 
        }
        return true;
    }

    // If packet is duplicated, ignore and wait for next
    if (mag == streamState.prevMag && ang == streamState.prevAng) return true;
    streamState.hasPrev = false; // reset baseline
    return false;
}

bool BinaryStreamProcessor::resyncStream(std::vector<uint8_t>& buffer) {
    Serial.println("Framing error detected. Sliding window to resync...");
    for (size_t i = 0; i < Serializer::RECORD_SIZE; ++i) {
        buffer.erase(buffer.begin());
        unsigned long startMillis = millis();
        while (UART_PORT.available() == 0) {
            if (millis() - startMillis > 200) {
                Serial.println("Timeout waiting for stream data.");
                return false; 
            }
        }
        buffer.push_back(UART_PORT.read());
        streamState.hasPrev = false;
        
        // Use the newly refactored test
        sumTestSerializedData(buffer); 
        
        std::vector<uint8_t> nextPacket;
        for (size_t j = 0; j < Serializer::RECORD_SIZE; ++j) {
            startMillis = millis();
            while (UART_PORT.available() == 0) {
                if (millis() - startMillis > 200) return false;
            }
            nextPacket.push_back(UART_PORT.read());
        }
        
        if (sumTestSerializedData(nextPacket)) {
            Serial.print("Sync recovered! Window shifted by ");
            Serial.print(i + 1);
            Serial.println(" bytes.");
            
            buffer = nextPacket; 
            return true;
        }
    }
    return false; 
}
