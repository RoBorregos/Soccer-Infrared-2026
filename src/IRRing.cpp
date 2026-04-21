// src/IRRing.cpp
// THIS WILL RUN ON THE ARDUINO UNO
#include "config.h"
#include "sensorControl.h"

// RoboCupJunior MODE-A waveform period: 833 us (1.2 kHz, 40 kHz carrier)
#define T_MODEA 833
#define T_READ_WINDOW (T_MODEA * 3)

// ─── Scan-cycle state ─────────────────────────────────────────────────────────
//
// lastPW[] is a "last known value" array — NOT a running accumulator.
// getAllSensorPulseWidthGroup() zeroes and re-measures only its 3 active slots
// on every call, so each slot always reflects the most recent 833 µs window
// in which that sensor was active.
//
// The key difference from the previous design:
//   OLD — accumulate 5 groups, then memset() all 15 slots.
//         The first vector after the reset is computed from 3 fresh readings
//         and 12 zeroes, spiking the result toward those 3 sensors.  This
//         repeats every ~4.2 ms and produces systematic, drift-like error.
//
//   NEW — no memset().  After each group the 12 non-active slots retain their
//         last measured value (max staleness = 4 × 833 µs = 3.3 ms).  The
//         vector is recomputed immediately after every group, giving ~240
//         estimates per second with no reset artifact.
//
static float   lastPW[IR_NUM];
static uint8_t currentGroup  = 0;
static uint8_t warmupGroups  = 0;   // counts up to SENSOR_GROUPS on first pass

#ifdef DEBUG_SMA
// SMA lives on the Arduino only in debug mode
static CircularMovingAverage smaForTheta(SMA_WINDOW_SIZE);
static unsigned long printTimer_ms = 0;
#endif

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(ARDUINO_BAUD);
    setAllSensorPinsInput();
    memset(lastPW, 0, sizeof(lastPW));
}

// Forward declarations
void emitTheta(float thetaDeg);

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // 1. Refresh only this group's 3 sensor slots
    getAllSensorPulseWidthGroup(lastPW, T_MODEA, currentGroup);
    currentGroup = (currentGroup + 1) % SENSOR_GROUPS;

    // 2. Gate output until every sensor has been measured at least once.
    //    Without this, the first SENSOR_GROUPS-1 vectors are computed from
    //    partially-initialised data (the remaining slots are still zero from
    //    the memset in setup()).
    if (warmupGroups < SENSOR_GROUPS) {
        warmupGroups++;
        return;
    }

    // 3. Compute bearing from all 15 last-known pulse widths
    vectorXY_t vectorXY = calcVectorXYFromPulseWidth(lastPW);
    vectorRT_t vectorRT  = calcRTfromXY(&vectorXY);

    emitTheta(vectorRT.theta);
}

// ─── Output ───────────────────────────────────────────────────────────────────
void emitTheta(float thetaDeg) {
#ifdef DEBUG_SMA
    // Debug: smooth on-chip and throtxtle to 20 Hz so a Serial Plotter can
    // keep up without flooding the buffer.
    float smoothed = smaForTheta.updateData(thetaDeg);
    if (millis() - printTimer_ms >= 50UL) {
        printTimer_ms = millis();
        Serial.println(smoothed, 1);
    }
#else
    // Production: send raw theta at full rate (~240 Hz).
    // The Teensy performs the CircularMovingAverage — keeping the Uno's loop()
    // as tight as possible so no measurement windows are skipped.
    // Format: "XXX.XX\n"  (max 8 bytes including newline — well within
    // 115200 baud capacity at 240 Hz).
    Serial.println(thetaDeg, 2);
#endif
}