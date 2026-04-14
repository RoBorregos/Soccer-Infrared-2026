#include "sensorControl.h"

void setAllSensorPinsInput(void) {
    for (int i = 0; i < IR_NUM; ++i) {
        pinMode(SensorPins[i], INPUT);
    }
}

// ─── getSensorPin ─────────────────────────────────────────────────────────────
// ATmega328P port map:
//
// Each case maps the sensor index to the correct register bit for the pin
// listed in SensorPins[].  Active LOW: sensor output is LOW when IR is detected.
//
// https://pro.easyeda.com/editor#id=53df9a819d7440a79b885182a97d962c,tab=*fa3ef932db0e420bb58348f4276d2377@53df9a819d7440a79b885182a97d962c

bool getSensorPin(uint8_t pin) {
    switch(pin) {
        case 1:  return PIND & (1<<2);  // PD2, Physical Pin 04
        case 2:  return PIND & (1<<3);  // PD3, Physical Pin 05
        case 3:  return PIND & (1<<5);  // PD5, Physical Pin 11
        case 4:  return PIND & (1<<6);  // PD6, Physical Pin 12
        case 5:  return PIND & (1<<7);  // PD7, Physical Pin 13
        
        // Port B (Digital Pins 8-13)
        case 6:  return PINB & (1<<0);  // PB0, Physical Pin 14
        case 7:  return PINB & (1<<1);  // PB1, Physical Pin 15
        case 8: return PINB & (1<<2);  // PB2, Physical Pin 16
        case 9: return PINB & (1<<4);  // PB4, Physical Pin 18
        case 10: return PINB & (1<<5);  // PB5, Physical Pin 19
        
        // Port C (Analog Pins A0-A5)
        case 11: return PINC & (1<<0);  // PC0, Physical Pin 23
        case 12: return PINC & (1<<1);  // PC1, Physical Pin 24
        case 13: return PINC & (1<<2);  // PC2, Physical Pin 25
        case 14: return PINC & (1<<3);  // PC3, Physical Pin 26
    }
}

// ─── getAllSensorPulseWidth ───────────────────────────────────────────────────
// Reads all IR_NUM sensors simultaneously for timeLimit_us microseconds.
// pulseWidth[i] accumulates the total time (in us) that sensor i was LOW
// (i.e. detecting the 40 kHz carrier).
void getAllSensorPulseWidth(float pulseWidth[IR_NUM], uint16_t timeLimit_us) {

    for (int i = 0; i < IR_NUM; i++) {
        pulseWidth[i] = 0.0f;
    }

    const unsigned long startTime_us = micros();
    unsigned long       prevTime_us  = startTime_us;

    do {
        unsigned long now_us = micros();
        float dt = (float)(now_us - prevTime_us);
        prevTime_us = now_us;
        for (int i = 0; i < IR_NUM; i++) {
            if (getSensorPin(i) == false) {   // active LOW
                pulseWidth[i] += dt;
            }
        }
    } while ((micros() - startTime_us) < timeLimit_us);
}

// ─── calcVectorXYFromPulseWidth ───────────────────────────────────────────────
// Quadratic weighting (WEIGHT_EXPONENT = 2) suppresses weak off-axis sensors,
// sharpening angular resolution without needing a physical sensor mask.
//
vectorXY_t calcVectorXYFromPulseWidth(float *pulseWidth) {
    vectorXY_t rslt = {0.0f, 0.0f};
    for (int i = 0; i < IR_NUM; i++) {
        float w = powf(pulseWidth[i], WEIGHT_EXPONENT);
        rslt.x += w * unitVectorX[i];
        rslt.y += w * unitVectorY[i];
    }
    return rslt;
}

// ─── calcRTfromXY ─────────────────────────────────────────────────────────────
// Converts XY vector to polar form.
// atan2(x, y) gives bearing measured clockwise from +Y (forward = 0 deg).
//
vectorRT_t calcRTfromXY(vectorXY_t *p) {
    vectorRT_t rslt;
    rslt.radius = sqrtf(p->x * p->x + p->y * p->y);
    rslt.theta  = atan2f(p->x, p->y) / (float)PI * 180.0f;
    return rslt;
}