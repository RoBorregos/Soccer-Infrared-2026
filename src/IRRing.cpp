// src/IRRing.cpp
// THIS WILL RUN ON THE ARDUINO UNO
#include "sensorControl.h"

// RoboCupJunior MODE-A waveform period: 833 us (1.2 kHz, 40 kHz carrier)
#define T_MODEA 833

// Set Uno to 57600. Due to Uno's 16MHz clock divider, this ACTUALLY produces 58824 baud.
#define BAUD 9600

// Theta: circular moving average — handles the +/-180 deg wrap correctly.
CircularMovingAverage smaForTheta(80);

unsigned long printTimer_ms = 0;

// ─── Scan-cycle state ─────────────────────────────────────────────────────────
// One full scan = SENSOR_GROUPS windows × T_MODEA each ≈ 5 × 833 µs = ~4.2 ms.
// Each window samples only the 3 sensors forming one equilateral triangle;
// all 15 duty cycles accumulate across the 5 windows before the vector is
// computed.
//
// smoothedTheta is declared here (not inside loop()) so the height correction
// can use the previous cycle's bearing estimate.  On the very first cycle it
// starts at 0°; the correction converges within one scan (~4.2 ms).
//
static float   accumulatedPW[IR_NUM];
static uint8_t currentGroup  = 0;
static float   smoothedTheta = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD);
    setAllSensorPinsInput();
    memset(accumulatedPW, 0, sizeof(accumulatedPW));
}

// Forward declarations
void serialPrintTheta(float thetaDeg);

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // 1. Measure the 3 sensors forming the current equilateral-triangle group.
    //    Duty cycle [0, 1] is written into the three active slots of
    //    accumulatedPW[].  The remaining slots are untouched.
    getAllSensorPulseWidthGroup(accumulatedPW, T_MODEA, currentGroup);
    currentGroup++;

    // 2. Once all 5 groups have been measured, the full ring is sampled.
    if (currentGroup >= SENSOR_GROUPS) {
        currentGroup = 0;

        // 3. Compensate for sensor height above ground before computing bearing.
        //    Uses smoothedTheta from the PREVIOUS full cycle as the bearing
        //    estimate (one-cycle lag ≈ 4.2 ms — negligible in practice).
        applyHeightCorrection(accumulatedPW, smoothedTheta);

        // 4. Weighted vector sum → raw bearing
        vectorXY_t vectorXY = calcVectorXYFromPulseWidth(accumulatedPW);
        vectorRT_t vectorRT  = calcRTfromXY(&vectorXY);

        // 5. Circular moving average smoothing
        smoothedTheta = smaForTheta.updateData(vectorRT.theta);

        // 6. Print at 20 Hz
        if (millis() - printTimer_ms >= 50UL) {
            printTimer_ms = millis();
            serialPrintTheta(smoothedTheta);
        }

        // 7. Clear accumulator for next full cycle
        memset(accumulatedPW, 0, sizeof(accumulatedPW));
    }
}

// ─── Serial helpers ───────────────────────────────────────────────────────────
void serialPrintTheta(float thetaDeg) {
    // Sends string data formatted as "X.X\r\n"
    Serial.println(thetaDeg, 1);
}