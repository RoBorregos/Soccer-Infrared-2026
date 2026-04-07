#ifndef __SENSOR_CONTROL__
#define __SENSOR_CONTROL__

#include "Arduino.h"
#define IR_NUM 15

/** 
 * IR Sensor Pins, Unit Vectors and Pulse Width Increment
 * https://www.desmos.com/calculator/ebr86ynq9p
 * 
*/
const uint8_t   SensorPins[IR_NUM]  = {19, 23, 24, 25, 11, 5, 4, 12, 13, 14, 15, 26, 16, 17, 18};
const float     unitVectorX[IR_NUM] = {0.0435, 0.4426, 0.7545, 0.9462, 0.9994, 0.9096, 0.6457, 0.2536, -0.2034, -0.6091, -0.8795, -0.9963, -0.9378, -0.7211, -0.3651};
const float     unitVectorY[IR_NUM] = {0.9991, 0.8967, 0.6563, 0.3236, -0.0346, -0.4155, -0.7636, -0.9673, -0.9791, -0.7931, -0.4759, -0.0859, 0.3471, 0.6928, 0.9310};
const float     deltaPulseWidth     = 2.0;

typedef struct {
    int activeSensors;      
    int maxPulseWidth;      
    int maxSensorNumber;    
    double avgPulseWidth;
} sensorInfo_t;

typedef struct {
    float x;
    float y;
} vectorXY_t;

typedef struct {
    float radius;
    float theta;
} vectorRT_t;

void setAllSensorPinsInput(void);
bool getSensorPin(uint8_t pin);
sensorInfo_t getAllSensorPulseWidth(float pulseWidth[IR_NUM], uint16_t timeLimit);
vectorXY_t calcVectorXYFromPulseWidth(float *pulseWidth);
vectorRT_t calcRTfromXY(vectorXY_t *vectorXY_p);

#endif
