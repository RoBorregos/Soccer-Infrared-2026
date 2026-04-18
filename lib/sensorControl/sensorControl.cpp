#include "sensorControl.h"

void setAllSensorPinsInput(void) {
    for (int i = 0; i < IR_NUM; ++i) {
        pinMode(SensorPins[i], INPUT);
    }
}

// ─── getSensorPin ─────────────────────────────────────────────────────────────
// Takes a 0-based sensor index (not an Arduino pin number).
// Active LOW: returns false when IR carrier is detected.
//
// Index → Arduino pin → AVR register bit
//   0 → D2  → PD2    5 → D8  → PB0   10 → D13 → PB5
//   1 → D3  → PD3    6 → D9  → PB1   11 → A0  → PC0
//   2 → D5  → PD5    7 → D10 → PB2   12 → A1  → PC1
//   3 → D6  → PD6    8 → D11 → PB3   13 → A2  → PC2
//   4 → D7  → PD7    9 → D12 → PB4   14 → A3  → PC3
//
bool getSensorPin(uint8_t sensorIndex) {
    switch (sensorIndex) {
        case  0: return PIND & (1 << 2);
        case  1: return PIND & (1 << 3);
        case  2: return PIND & (1 << 5);
        case  3: return PIND & (1 << 6);
        case  4: return PIND & (1 << 7);
        case  5: return PINB & (1 << 0);
        case  6: return PINB & (1 << 1);
        case  7: return PINB & (1 << 2);
        case  8: return PINB & (1 << 3);
        case  9: return PINB & (1 << 4);
        case 10: return PINB & (1 << 5);
        case 11: return PINC & (1 << 0);
        case 12: return PINC & (1 << 1);
        case 13: return PINC & (1 << 2);
        case 14: return PINC & (1 << 3);
        default: return false;
    }
}

// ─── getAllSensorPulseWidthGroup ──────────────────────────────────────────────
// Samples the 3 sensors of the given triangle group for timeLimit_us
// microseconds, then stores each sensor's duty cycle ∈ [0, 1] into
// pulseWidth[].  Only the three active slots are written; the caller owns
// the rest of the array.
//
// ── Drift fix: sample counting replaces dt-accumulation ──────────────────────
// Root cause of the previous drift: micros() on ATmega328P is updated by a
// Timer0 overflow ISR.  When Serial, I2C, or other ISRs delay the overflow
// handler, consecutive micros() calls inside the measurement loop return the
// same value (dt = 0) for several iterations, then produce a large catch-up
// dt when the ISR finally fires.  The resulting Σdt drifts non-constantly
// depending on ISR interference patterns — exactly the symptom reported.
//
// Fix: instead of accumulating dt, count the number of loop iterations
// where each sensor is LOW (carrier detected), then divide by total
// iterations.  This yields duty cycle = t_LOW / t_window regardless of
// absolute loop speed.  No timing arithmetic inside the hot path.
// The only micros() call that remains is in the while condition, which
// purely gates the exit — it does not influence the measurement values.
//
// pulseWidth values are now duty cycle [0, 1].  calcVectorXYFromPulseWidth()
// uses powf(pulseWidth[i], WEIGHT_EXPONENT), so relative magnitudes are
// preserved and the vector math is unchanged.
//
void getAllSensorPulseWidthGroup(float pulseWidth[IR_NUM],
                                 uint16_t timeLimit_us,
                                 uint8_t groupOffset) {
    uint16_t lowCount[SENSOR_GROUP_SIZE] = {};
    uint16_t totalCount = 0;

    const unsigned long startTime_us = micros();
    do {
        for (int g = 0; g < SENSOR_GROUP_SIZE; g++) {
            uint8_t idx = groupOffset + (uint8_t)(g * SENSOR_GROUPS);
            if (!getSensorPin(idx)) {
                lowCount[g]++;
            }
        }
        totalCount++;
    } while ((micros() - startTime_us) < timeLimit_us);

    const float norm = (totalCount > 0) ? (1.0f / (float)totalCount) : 0.0f;
    for (int g = 0; g < SENSOR_GROUP_SIZE; g++) {
        uint8_t idx = groupOffset + (uint8_t)(g * SENSOR_GROUPS);
        pulseWidth[idx] = (float)lowCount[g] * norm;   // duty cycle ∈ [0, 1]
    }
}

// ─── applyHeightCorrection ────────────────────────────────────────────────────
// Compensates each sensor's measured duty cycle for the reduction in received
// power caused by the ring being at height h above the ground plane.
//
// Model (cosine-response sensor, optical axis horizontal):
//   P_elevated ∝  d_horiz / (d_horiz² + h²)^(3/2)
//   P_ground   ∝  1 / d_horiz²
//
// Correction factor per sensor:
//   k_i = (1 + h² / d_horiz_i²)^(3/2)  →  pulseWidth[i] *= k_i
//
// d_horiz_i is estimated from the previous bearing (thetaEst_deg) plus the
// known ring geometry, using the law of cosines:
//   d_horiz_i² = D² + R² − 2·D·R·cos(θ − φ_i)
//
// where D = BALL_DIST_EST_CM, R = RING_RADIUS_CM, and φ_i is the bearing of
// sensor i — recovered cheaply from the unit vectors already in the header:
//   cos(θ − φ_i) = sinθ · unitVectorX[i] + cosθ · unitVectorY[i]
//
// This costs 2 trig calls (sin/cos of θ) + 15 multiplies + 15 sqrtf calls per
// full scan cycle (~4.2 ms), which is well within Uno budget.
//
// One-cycle lag (θ_est from previous cycle ≈ 4.2 ms old) is negligible.
//
void applyHeightCorrection(float pulseWidth[IR_NUM], float thetaEst_deg) {
    const float h2    = SENSOR_HEIGHT_CM * SENSOR_HEIGHT_CM;
    const float D     = BALL_DIST_EST_CM;
    const float R     = RING_RADIUS_CM;
    const float D2    = D * D;
    const float R2    = R * R;
    const float twoRD = 2.0f * R * D;

    // Bearing direction as (sinθ, cosθ) in the atan2(x,y) convention used
    // by calcRTfromXY — i.e. clockwise from +y.
    const float theta_rad = thetaEst_deg * ((float)M_PI / 180.0f);
    const float sinT = sinf(theta_rad);
    const float cosT = cosf(theta_rad);

    for (int i = 0; i < IR_NUM; i++) {
        if (pulseWidth[i] < 1e-6f) continue;   // skip sensors that saw nothing

        // cos(θ − φ_i) via dot product with sensor i's unit direction vector.
        // In this code's convention: sensor bearing φ_i satisfies
        //   unitVectorX[i] = sin(φ_i),  unitVectorY[i] = cos(φ_i)
        // so  (sinθ, cosθ) · (sinφ_i, cosφ_i) = cos(θ − φ_i)  ✓
        const float cos_delta = sinT * unitVectorX[i] + cosT * unitVectorY[i];

        // Horizontal distance from sensor i to estimated ball position
        float d2 = D2 + R2 - twoRD * cos_delta;
        if (d2 < 4.0f) d2 = 4.0f;   // clamp to 2 cm minimum (avoids div/0)

        // k = (1 + h²/d²)^(3/2)  computed as  ratio * sqrt(ratio)
        // to avoid powf with non-integer exponent.
        const float ratio = 1.0f + h2 / d2;       // = (r/d)²
        pulseWidth[i] *= ratio * sqrtf(ratio);     // = (r/d)³
    }
}

// ─── calcVectorXYFromPulseWidth ───────────────────────────────────────────────
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
// atan2f(x, y) → bearing clockwise from +y (forward = 0°).
vectorRT_t calcRTfromXY(vectorXY_t *p) {
    vectorRT_t rslt;
    rslt.radius = sqrtf(p->x * p->x + p->y * p->y);
    rslt.theta  = atan2f(p->x, p->y) / (float)PI * 180.0f;
    return rslt;
}