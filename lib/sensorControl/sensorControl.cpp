#include "sensorControl.h"

void setAllSensorPinsInput(void) {
    for (int i = 0; i < IR_NUM; ++i) {
        pinMode(SensorPins[i], INPUT);
    }
}

// ─── getSensorPin ─────────────────────────────────────────────────────────────
// ATmega328P port map — takes a 0-based sensor INDEX (not an Arduino pin number).
//
// Index → Arduino pin → AVR register bit
//   0 → D2  → PD2    5 → D8  → PB0   10 → D13 → PB5
//   1 → D3  → PD3    6 → D9  → PB1   11 → A0  → PC0
//   2 → D5  → PD5    7 → D10 → PB2   12 → A1  → PC1
//   3 → D6  → PD6    8 → D11 → PB3   13 → A2  → PC2
//   4 → D7  → PD7    9 → D12 → PB4   14 → A3  → PC3
//
// Active LOW: sensor output is LOW when IR carrier is detected.
//
bool getSensorPin(uint8_t sensorIndex) {
    switch (sensorIndex) {
        case  0: return PIND & (1 << 2);  // PD2, Physical Pin 04
        case  1: return PIND & (1 << 3);  // PD3, Physical Pin 05
        case  2: return PIND & (1 << 5);  // PD5, Physical Pin 11
        case  3: return PIND & (1 << 6);  // PD6, Physical Pin 12
        case  4: return PIND & (1 << 7);  // PD7, Physical Pin 13
        case  5: return PINB & (1 << 0);  // PB0, Physical Pin 14
        case  6: return PINB & (1 << 1);  // PB1, Physical Pin 15
        case  7: return PINB & (1 << 2);  // PB2, Physical Pin 16
        case  8: return PINB & (1 << 3);  // PB3, Physical Pin 17
        case  9: return PINB & (1 << 4);  // PB4, Physical Pin 18
        case 10: return PINB & (1 << 5);  // PB5, Physical Pin 19
        case 11: return PINC & (1 << 0);  // PC0, Physical Pin 23
        case 12: return PINC & (1 << 1);  // PC1, Physical Pin 24
        case 13: return PINC & (1 << 2);  // PC2, Physical Pin 25
        case 14: return PINC & (1 << 3);  // PC3, Physical Pin 26
        default: return false;
    }
}

// ─── getAllSensorPulseWidthGroup ──────────────────────────────────────────────
// Measures ONLY the 3 sensors forming the equilateral triangle for groupOffset.
// Those 3 slots in pulseWidth[] are zeroed then written fresh.
// All other slots are left untouched — the caller retains last-known values.
//
//   groupOffset 0 → sensors  0,  5, 10  (  0°, 120°, 240°)
//   groupOffset 1 → sensors  1,  6, 11  ( 24°, 144°, 264°)
//   groupOffset 2 → sensors  2,  7, 12  ( 48°, 168°, 288°)
//   groupOffset 3 → sensors  3,  8, 13  ( 72°, 192°, 312°)
//   groupOffset 4 → sensors  4,  9, 14  ( 96°, 216°, 336°)
//
void getAllSensorPulseWidthGroup(float pulseWidth[IR_NUM],
                                 uint16_t timeLimit_us,
                                 uint8_t groupOffset) {
    for (int g = 0; g < SENSOR_GROUP_SIZE; g++) {
        pulseWidth[groupOffset + (uint8_t)(g * SENSOR_GROUPS)] = 0.0f;
    }

    const unsigned long startTime_us = micros();
    unsigned long       prevTime_us  = startTime_us;

    do {
        unsigned long now_us = micros();
        float dt = (float)(now_us - prevTime_us);
        prevTime_us = now_us;

        for (int g = 0; g < SENSOR_GROUP_SIZE; g++) {
            uint8_t idx = groupOffset + (uint8_t)(g * SENSOR_GROUPS);
            if (!getSensorPin(idx)) {   // active LOW → carrier detected
                pulseWidth[idx] += dt;
            }
        }
    } while ((micros() - startTime_us) < timeLimit_us);
}

// ─── calcVectorXYFromPulseWidth ───────────────────────────────────────────────
vectorXY_t calcVectorXYFromPulseWidth(float *pulseWidth) {
    vectorXY_t rslt = {0.0f, 0.0f};
    for (int i = 0; i < IR_NUM; i++) {
        // Using powf with a lower exponent
        float w = powf(pulseWidth[i], WEIGHT_EXPONENT);
        rslt.x += w * unitVectorX[i];
        rslt.y += w * unitVectorY[i];
    }
    return rslt;
}

// ─── calcRTfromXY ─────────────────────────────────────────────────────────────
// atan2f(x, y) gives bearing clockwise from +Y (forward = 0 deg).
vectorRT_t calcRTfromXY(vectorXY_t *p) {
    vectorRT_t rslt;
    rslt.radius = sqrtf(p->x * p->x + p->y * p->y);
    rslt.theta  = atan2f(p->x, p->y) / (float)PI * 180.0f;
    return rslt;
}