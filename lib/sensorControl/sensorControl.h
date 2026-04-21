#ifndef __SENSORCONTROL__
#define __SENSORCONTROL__

#include <Arduino.h>
#include "config.h"
#include "moving_average.h"

#define IR_NUM            15
#define SENSOR_GROUP_SIZE  3                           // sensors active per window
#define SENSOR_GROUPS     (IR_NUM / SENSOR_GROUP_SIZE) // = 5 rotating triangle groups

const uint8_t SensorPins[IR_NUM] = {2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3};

// Unit vectors for 15 sensors at 360/15 = 24 deg spacing.
// https://www.desmos.com/calculator/hvkq5oqwmf
const float unitVectorX[IR_NUM] = {
     0.926,  0.843,  0.591,  0.233, -0.188,
    -0.556, -0.816, -0.923, -0.870, -0.668,
    -0.337,  0.044,  0.405,  0.703,  0.874
};
const float unitVectorY[IR_NUM] = {
     0.036,  0.385,  0.714,  0.897,  0.908,
     0.741,  0.440,  0.078, -0.320, -0.642,
    -0.864, -0.926, -0.833, -0.604, -0.308
};

// Sharpens angular resolution by suppressing weak off-axis sensor contributions.
// 1.0 = linear, 2.0 = quadratic (recommended), 3.0 = approaches max-sensor-wins.
#define WEIGHT_EXPONENT 2.0f

typedef struct { float x; float y;          } vectorXY_t;
typedef struct { float radius; float theta; } vectorRT_t;

// ─── Public API ───────────────────────────────────────────────────────────────
void         setAllSensorPinsInput(void);
bool         getSensorPin(uint8_t sensorIndex);                 // 0-indexed
void         getAllSensorPulseWidthGroup(float pulseWidth[IR_NUM],
                                        uint16_t timeLimit_us,
                                        uint8_t groupOffset);  // 0..(SENSOR_GROUPS-1)
vectorXY_t   calcVectorXYFromPulseWidth(float *pulseWidth);
vectorRT_t   calcRTfromXY(vectorXY_t *vectorXY_p);

#endif // __SENSORCONTROL__