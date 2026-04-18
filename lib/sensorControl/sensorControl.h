#ifndef __SENSORCONTROL__
#define __SENSORCONTROL__

#include <Arduino.h>
#include "moving_average.h"

#define IR_NUM            15
#define SENSOR_GROUP_SIZE  3
#define SENSOR_GROUPS     (IR_NUM / SENSOR_GROUP_SIZE)  // = 5

const uint8_t SensorPins[IR_NUM] = {2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3};

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

#define WEIGHT_EXPONENT 2.0f

// ─── Height correction parameters ─────────────────────────────────────────────
//
// The sensor ring sits at SENSOR_HEIGHT_CM above the ground plane.  A
// horizontally-aimed TSOP sensor sees the ball at elevation angle
//   α = atan(h / d_horiz)  below its optical axis.
//
// For a cosine-response sensor the received power is:
//   P_elev  ∝  d_horiz / (d_horiz² + h²)^(3/2)
//
// vs. a ground-level sensor:
//   P_ground ∝  1 / d_horiz²
//
// The correction factor k = (1 + h²/d_horiz²)^(3/2) restores ground-plane
// weighting.  Without it, on-axis sensors (closest to the ball) are penalised
// MORE than off-axis ones, broadening the effective beam and biasing the vector
// sum — the effect is strongest at close range where α is large.
//
// SENSOR_HEIGHT_CM : measure on the robot.
// BALL_DIST_EST_CM : set to the typical playing distance; at 50 cm k ≈ 1.07,
//                   at 20 cm k ≈ 1.47, at 10 cm k ≈ 3.2.  Tune accordingly.
// RING_RADIUS_CM   : horizontal radius of the sensor ring; measure on the robot.
//
#define SENSOR_HEIGHT_CM  10.8f
#define BALL_DIST_EST_CM  50.0f
#define RING_RADIUS_CM     4.5f

typedef struct { float x; float y;          } vectorXY_t;
typedef struct { float radius; float theta; } vectorRT_t;

// ─── Public API ───────────────────────────────────────────────────────────────
void         setAllSensorPinsInput(void);
bool         getSensorPin(uint8_t sensorIndex);                               // 0-indexed
void         getAllSensorPulseWidthGroup(float pulseWidth[IR_NUM],
                                         uint16_t timeLimit_us,
                                         uint8_t groupOffset);
void         applyHeightCorrection(float pulseWidth[IR_NUM], float thetaEst_deg);
vectorXY_t   calcVectorXYFromPulseWidth(float *pulseWidth);
vectorRT_t   calcRTfromXY(vectorXY_t *vectorXY_p);

#endif // __SENSORCONTROL__