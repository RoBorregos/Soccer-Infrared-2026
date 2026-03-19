#include "moveifball.h"

MoveIfBall::MoveIfBall(HardwareSerial& inputSerial, HardwareSerial& debugSerial)
	: inputSerial(&inputSerial),
	  debugSerial(&debugSerial) {}

void MoveIfBall::begin(unsigned long baudRate) {
	inputSerial->begin(baudRate);
}

bool MoveIfBall::ballDetected() {
	if (!inputSerial->available()) {
		debugSerial->println("Did not detect anything");
		return false;
	}

	while (inputSerial->available()) {
		const char incomingData = static_cast<char>(inputSerial->read());

		if (incomingData == 'M') {
			return true;
		}
	}

	debugSerial->println("Did not detect anything");
	return false;
}
