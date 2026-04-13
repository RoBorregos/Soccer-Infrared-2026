#include "sensor_control.h"

/**
 * PinMode for all IR pins
 */
void setAllSensorPinsInput(void) {
    for (int i = 0; i < IR_NUM; ++i) {
        pinMode(SensorPins[i], INPUT);
    }
}

/** 
 * Get values from registers directly for faster access
 * https://docs.arduino.cc/retired/hacking/hardware/PinMapping168/
 * */
bool getSensorPin(uint8_t pin) {
    switch(pin) {
        // 2, 3, 6, 7, 8
        case 0:  return PINB&(1<<5);  // Physical Pin 19
        case 1:  return PINC&(1<<0);  // Physical Pin 23
        case 2:  return PINC&(1<<1);  // Physical Pin 24
        case 3:  return PINC&(1<<2);  // Physical Pin 25
        case 4:  return PIND&(1<<5);  // Physical Pin 11
        case 5:  return PIND&(1<<3);  // Physical Pin 05
        case 6:  return PIND&(1<<2);  // Physical Pin 04
        case 7:  return PIND&(1<<6);  // Physical Pin 12
        case 8:  return PIND&(1<<7);  // Physical Pin 13
        case 9:  return PINB&(1<<0);  // Physical Pin 14
        case 10: return PINB&(1<<1);  // Physical Pin 15
        case 11: return PINC&(1<<3);  // Physical Pin 26
        case 12: return PINB&(1<<2);  // Physical Pin 16
        case 13: return PINB&(1<<3);  // Physical Pin 17
        case 14: return PINB&(1<<4);  // Physical Pin 18
    }
}

/**
 * Get pulse widths from all sensors within a time limit
 */
sensorInfo_t getAllSensorPulseWidth(float pulseWidth[IR_NUM], uint16_t timeLimit) {
    sensorInfo_t sensorInfo;

    for(int i = 0; i < IR_NUM; i++) {
        pulseWidth[i] = 0;
    }    


    const unsigned long startTime_us = micros();
    do {
        for (int i = 0; i < IR_NUM; i++) {
            if(getSensorPin(i) == false) {
                pulseWidth[i] += deltaPulseWidth;
            }
        }
    } while((micros() - startTime_us) < timeLimit);

    sensorInfo.activeSensors   = 0; 
    sensorInfo.maxPulseWidth   = 0; 
    sensorInfo.maxSensorNumber = 0; 
    sensorInfo.avgPulseWidth   = 0;
    
    for(int i = 0; i < IR_NUM; i++) {
        if(pulseWidth[i] > 0) {
            sensorInfo.activeSensors += 1;
            sensorInfo.avgPulseWidth += pulseWidth[i];
        }
        if(pulseWidth[i] > sensorInfo.maxPulseWidth) {
            sensorInfo.maxPulseWidth = pulseWidth[i];
            sensorInfo.maxSensorNumber = i;
        }
    }


    return sensorInfo;
}

/**
 * Calculate vector components from pulse widths
 */
vectorXY_t calcVectorXYFromPulseWidth(float *pulseWidth) {
    vectorXY_t rslt = {0, 0};
    for(int i = 0; i < IR_NUM; i++) {
        rslt.x += pulseWidth[i] * unitVectorX[i];
        rslt.y += pulseWidth[i] * unitVectorY[i];
    }

    return rslt;
}

/**
 * Radius calculation (Pythagorean theorem)
 * Angle calculation (atan2 in degrees)
*/
vectorRT_t calcRTfromXY(vectorXY_t *vectorXY_p) {
    vectorRT_t rslt;
    rslt.radius  = sqrt(pow(vectorXY_p->x, 2.0) + pow(vectorXY_p->y, 2.0));
    rslt.theta   = atan2(vectorXY_p->x, vectorXY_p->y) / PI * 180.0;

    return rslt;
}
