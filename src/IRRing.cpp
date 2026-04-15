// src/IRRing.cpp
// THIS WILL RUN ON THE ARDUINO UNO
#include "sensorcontrol.h"

// RoboCupJunior MODE-A waveform period: 833 us (1.2 kHz, 40 kHz carrier)
#define T_MODEA 833

// Uno-side UART baud for IR telemetry to the Teensy.
#define BAUD 9600

// Theta: circular moving average handles the +/-180 deg wrap correctly.
CircularMovingAverage smaForTheta(80);

unsigned long printTimer_ms = 0;

void serialPrintIRData(float thetaDeg, float strength);

void setup() {
    Serial.begin(BAUD);
    setAllSensorPinsInput();
}

void loop() {
    float pulseWidth[IR_NUM];
    vectorXY_t vectorXY;
    vectorRT_t vectorRT;

    getAllSensorPulseWidth(pulseWidth, T_MODEA);
    vectorXY = calcVectorXYFromPulseWidth(pulseWidth);
    vectorRT = calcRTfromXY(&vectorXY);

    const float smoothedTheta = smaForTheta.updateData(vectorRT.theta);

    if (millis() - printTimer_ms >= 50UL) {
        printTimer_ms = millis();
        serialPrintIRData(smoothedTheta, vectorRT.radius);
    }
}

void serialPrintIRData(float thetaDeg, float strength) {
    Serial.print("a ");
    Serial.println(thetaDeg, 1);
    Serial.print("r ");
    Serial.println(strength, 2);
}
