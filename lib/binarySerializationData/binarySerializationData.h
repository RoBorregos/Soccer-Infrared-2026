/**
 * The objective is to have communication between the auxiliary arduino
 * and the main arduino. It's important to only use 32 bits of data on 
 * my main struct, as I want for Serial communication to pass seamlessly.
 * 
 * This is how the data will be structured:
 * - ballMagnitude: 16 bits (0 to 65535)
 * - ballAngle: 16 bits (0 to 65535)
 * 
 * These will be mapped in such a way to fit expected ranges:
 * - ballMagnitude: 0 to 100 (mapped to 0 to 65535)
 * - ballAngle: -180 to 180 (mapped to 0 to 65535)
*/

#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

struct IRSerializationData {
    uint16_t ballMagnitude;
    uint16_t ballAngle; 
}__attribute__((packed)); // Ensure no padding is added by the compiler

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(IRSerializationData);
