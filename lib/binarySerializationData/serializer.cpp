#include "serializer.h"

size_t Serializer::serializeTo(const IRSerializationData& data, uint8_t* dest, size_t destSize) {
    if (!dest || destSize < Serializer::RECORD_SIZE) return 0;
    // pack as big-endian: [mag high][mag low][angle high][angle low]
    uint16_t mag = data.ballMagnitude;
    uint16_t ang = data.ballAngle;
    dest[0] = static_cast<uint8_t>((mag >> 8) & 0xFF);
    dest[1] = static_cast<uint8_t>(mag & 0xFF);
    dest[2] = static_cast<uint8_t>((ang >> 8) & 0xFF);
    dest[3] = static_cast<uint8_t>(ang & 0xFF);
    return Serializer::RECORD_SIZE;
}

std::vector<uint8_t> Serializer::serialize(const IRSerializationData& data) {
    std::vector<uint8_t> out(Serializer::RECORD_SIZE);
    serializeTo(data, out.data(), out.size());
    return out;
}

bool Serializer::deserializeFrom(const uint8_t* buffer, size_t size, IRSerializationData& out) {
    if (!buffer || size < Serializer::RECORD_SIZE) return false;
    // read big-endian
    uint16_t mag = (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1]);
    uint16_t ang = (static_cast<uint16_t>(buffer[2]) << 8) | static_cast<uint16_t>(buffer[3]);
    out.ballMagnitude = mag;
    out.ballAngle = ang;
    return true;
}

std::vector<IRSerializationData> Serializer::deserialize(const uint8_t* buffer, size_t size) {
    std::vector<IRSerializationData> out;
    if (!buffer || size < Serializer::RECORD_SIZE) return out;
    size_t count = size / Serializer::RECORD_SIZE;
    out.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        IRSerializationData d;
        deserializeFrom(buffer + i * Serializer::RECORD_SIZE, Serializer::RECORD_SIZE, d);
        out.push_back(d);
    }
    return out;
}

std::vector<IRSerializationData> Serializer::deserialize(const std::vector<uint8_t>& buffer) {
    return deserialize(buffer.data(), buffer.size());
}