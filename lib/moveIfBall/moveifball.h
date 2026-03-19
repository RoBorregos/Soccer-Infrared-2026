#pragma once

#include <Arduino.h>

class MoveIfBall {
public:
	explicit MoveIfBall(HardwareSerial& inputSerial = Serial, HardwareSerial& debugSerial = Serial);

	void begin(unsigned long baudRate);
	bool ballDetected();

private:
	HardwareSerial* inputSerial;
	HardwareSerial* debugSerial;
};
