// src/IRRing.cpp
// THIS WILL RUN ON THE ARDUINO UNO
#include "sensorcontrol.h"

// RoboCupJunior MODE-A waveform period: 833 us (1.2 kHz, 40 kHz carrier)
#define T_MODEA 833
#define T_READ_WINDOW (T_MODEA * 5)
// Set Uno to 57600. Due to Uno's 16MHz clock divider, this ACTUALLY produces 58824 baud.
#define BAUD 9600 

// Theta: circular moving average — handles the +/-180 deg wrap correctly.
CircularMovingAverage smaForTheta(80);

unsigned long printTimer_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD);
    setAllSensorPinsInput();
}

// Forward declarations
void serialPrintTheta(float thetaDeg);

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    float        pulseWidth[IR_NUM];
    vectorXY_t   vectorXY;
    vectorRT_t   vectorRT;

    // 1. Read all sensor pulse widths simultaneously over the extended window
    getAllSensorPulseWidth(pulseWidth, T_READ_WINDOW);

    // 2. Compute bearing (theta) via weighted vector sum
    vectorXY = calcVectorXYFromPulseWidth(pulseWidth);
    vectorRT = calcRTfromXY(&vectorXY);

    // 3. Smooth theta output
    float smoothedTheta      = smaForTheta.updateData(vectorRT.theta);

    // 4. Serial print at 50 ms / 20 Hz
    if (millis() - printTimer_ms >= 50UL) {
        printTimer_ms = millis();
        serialPrintTheta(smoothedTheta);
    }
}

// ─── Serial helpers ───────────────────────────────────────────────────────────
void serialPrintTheta(float thetaDeg) {
    // Sends string data formatted as "X.X\r\n"
    Serial.println(thetaDeg, 1);
}