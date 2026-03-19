#include "goodSerializer.h"
#define MAX_ATTEMPTS 5
#define RX 0
#define TX 1

#if !defined(__AVR_ATmega2560__)
#include <SoftwareSerial.h>
SoftwareSerial UART_PORT(RX, TX); // For main Arduino.
#endif

void GoodSerializer::begin(uint32_t serialBaud, uint32_t uartBaud) {
    Serial.begin(serialBaud);
    UART_PORT.begin(uartBaud);
    Serial.println("Main arduino is ready and listening...");
    Serial.println("Live stream test armed: validating received packets progress by +1/+1.");
}

void GoodSerializer::processLoop() {
    // Read all available bytes into the buffer (non-blocking)
    while (UART_PORT.available() > 0) {
        receiveBuffer.push_back(static_cast<uint8_t>(UART_PORT.read()));
    }

    if (receiveBuffer.size() < Serializer::RECORD_SIZE) return;
    std::vector<uint8_t> packet(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);

    // Helper: try to recover sync by sliding the window up to RECORD_SIZE bytes.
    auto trySlidingSync = [&](std::vector<uint8_t>& outPacket) -> bool {
        for (size_t shift = 1; shift <= Serializer::RECORD_SIZE && receiveBuffer.size() >= Serializer::RECORD_SIZE; ++shift) {
            // drop oldest byte and attempt to form a new candidate
            receiveBuffer.erase(receiveBuffer.begin());
            if (receiveBuffer.size() < Serializer::RECORD_SIZE) break;

            std::vector<uint8_t> candidate(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);
            streamState.hasPrev = false; // allow new baseline
            if (testSerializedData(candidate)) {
                Serial.print("Sync recovered! Window shifted by ");
                Serial.print(shift);
                Serial.println(" bytes.");
                // consume the validated packet from the buffer and report it
                receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);
                outPacket = std::move(candidate);
                return true;
            }
        }
        return false;
    };

    if (!testSerializedData(packet)) {
        if (!trySlidingSync(packet)) {
            // keep buffer bounded to avoid unbounded growth
            const size_t maxKeep = Serializer::RECORD_SIZE * 4;
            if (receiveBuffer.size() > maxKeep) {
                receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + (receiveBuffer.size() - maxKeep));
            }
            return;
        }
    } else {
        // Valid packet -> consume it
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + Serializer::RECORD_SIZE);
    }

    std::vector<IRSerializationData> deserialized = Serializer::deserialize(packet);
    if (deserialized.empty()) {
        Serial.println("Deserialization failed or empty packet");
        return;
    }

    Serial.print("M=");
    Serial.print(deserialized[0].ballMagnitude);
    Serial.print(" ang=");
    Serial.print(deserialized[0].ballAngle);

    Serial.print("\t| Serialized data: ");
    for (uint8_t byte : packet) {
        if (byte < 0x10) Serial.print("0");
        Serial.print(byte, HEX);
        Serial.print(" ");
    }
    Serial.println();
}

uint16_t GoodSerializer::readBE16(uint8_t hi, uint8_t lo) {
    return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

bool GoodSerializer::testSerializedData(const std::vector<uint8_t>& packet) {
    if (packet.size() < Serializer::RECORD_SIZE) return false;
    const uint16_t mag = readBE16(packet[0], packet[1]);
    const uint16_t ang = readBE16(packet[2], packet[3]);

    if (!streamState.hasPrev) {
        streamState.hasPrev = true;
        streamState.prevMag = mag;
        streamState.prevAng = ang;
        return true;
    }

    const uint16_t expectedMag = static_cast<uint16_t>(streamState.prevMag + 1);
    const uint16_t expectedAng = static_cast<uint16_t>(streamState.prevAng + 1);

    if (mag == expectedMag && ang == expectedAng) {
        streamState.prevMag = mag;
        streamState.prevAng = ang;
        ++streamState.checkedTransitions;

        if (!streamState.passPrinted && streamState.checkedTransitions >= MAX_ATTEMPTS) {
            Serial.println("Stream test passed: 5 consecutive +1/+1 transitions validated.");
            streamState.passPrinted = true;
        }

        return true;
    }

    if (mag == streamState.prevMag && ang == streamState.prevAng) {
        return true;
    }

    Serial.println("Stream test failed: packet progression is not +1/+1.");
    Serial.print("Prev M="); Serial.print(streamState.prevMag);
    Serial.print(" ang="); Serial.println(streamState.prevAng);
    Serial.print("Curr M="); Serial.print(mag);
    Serial.print(" ang="); Serial.println(ang);
    Serial.print("Expected M="); Serial.print(expectedMag);
    Serial.print(" ang="); Serial.println(expectedAng);
    Serial.print("Raw bytes: ");
    for (size_t i = 0; i < Serializer::RECORD_SIZE; ++i) {
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    return false;
}

bool GoodSerializer::resyncStream(std::vector<uint8_t>& buffer) {
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
        testSerializedData(buffer); 
        
        std::vector<uint8_t> nextPacket;
        for (size_t j = 0; j < Serializer::RECORD_SIZE; ++j) {
            startMillis = millis();
            while (UART_PORT.available() == 0) {
                if (millis() - startMillis > 200) return false;
            }
            nextPacket.push_back(UART_PORT.read());
        }
        
        if (testSerializedData(nextPacket)) {
            Serial.print("Sync recovered! Window shifted by ");
            Serial.print(i + 1);
            Serial.println(" bytes.");
            
            buffer = nextPacket; 
            return true;
        }
    }
    return false; 
}

bool GoodSerializer::sumTestSerializedData(const std::vector<uint8_t>& packet, uint16_t expectedMag, uint16_t expectedAng) {
    if (packet.size() < Serializer::RECORD_SIZE) return false;

    uint16_t mag = readBE16(packet[0], packet[1]);
    uint16_t ang = readBE16(packet[2], packet[3]);

    if (mag == expectedMag && ang == expectedAng) {
        return true;
    }

    Serial.println("Sum test failed: values do not match expected.");
    Serial.print("Expected M="); Serial.print(expectedMag);
    Serial.print(" ang="); Serial.println(expectedAng);
    Serial.print("Curr M="); Serial.print(mag);
    Serial.print(" ang="); Serial.println(ang);
    Serial.print("Raw bytes: ");
    for (size_t i = 0; i < Serializer::RECORD_SIZE; ++i) {
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    return false;
}