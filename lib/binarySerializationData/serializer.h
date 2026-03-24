#pragma once 
/**
 * Based on AIZER-2025 serializer. Replaced std::optional with bool
 * return values due to arduino limitations. 
*/

#include "binarySerializationData.h"
#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstring>

class Serializer {
public:
	Serializer() = delete;
	~Serializer() = delete;
	Serializer(const Serializer&) = delete;
	Serializer& operator=(const Serializer&) = delete;

	static constexpr size_t RECORD_SIZE = BINARY_SERIALIZATION_DATA_SIZE;

	// Serialize a single record into a provided buffer. Returns number of bytes written (0 on failure).
	static size_t serializeTo(const IRSerializationData& data, uint8_t* dest, size_t destSize);
	// Serialize a single record into a std::vector (convenience wrapper).
	static std::vector<uint8_t> serialize(const IRSerializationData& data);
    
	// Deserialize a single record from a provided buffer. Returns true on success.
	static bool deserializeFrom(const uint8_t* buffer, size_t size, IRSerializationData& out);
	// Deserialize all whole records from a buffer into a vector. Partial tail bytes are ignored.
	static std::vector<IRSerializationData> deserialize(const uint8_t* buffer, size_t size);
	// Convenience wrapper: deserialize from std::vector<uint8_t>
	static std::vector<IRSerializationData> deserialize(const std::vector<uint8_t>& buffer);
};