#ifndef __MOVING_AVERAGE__
#define __MOVING_AVERAGE__

#include <stdint.h>
#include <math.h>

// ─── CircularMovingAverage ────────────────────────────────────────────────────
// Moving average for angles (degrees), robust to the +/-180 degree discontinuity.
//
// Uses the directional-statistics circular mean:
//   1. Store each angle as a unit vector (cos theta, sin theta).
//   2. Average the cos and sin components separately.
//   3. Recover the mean via atan2(mean_sin, mean_cos).
// Reference: Mardia & Jupp, "Directional Statistics", Wiley 2000, S2.2.
//
class CircularMovingAverage {
private:
    float    *cosArray;
    float    *sinArray;
    uint16_t  sizeOfDataArray;
    uint16_t  count;
    bool      filled;
    float     sumC;
    float     sumS;

    static constexpr float DEG2RAD = M_PI / 180.0f;
    static constexpr float RAD2DEG = 180.0f / M_PI;

public:
    CircularMovingAverage(uint16_t num_array) {
        cosArray        = new float[num_array]();
        sinArray        = new float[num_array]();
        sizeOfDataArray = num_array;
        count           = 0;
        filled          = false;
        sumC = 0.0f;
        sumS = 0.0f;
    }

    // Returns the circular mean in degrees (-180 to +180).
    // Optimized O(1) update: maintain running sums so each update does
    // only a constant number of operations regardless of window size.
    float updateData(float angleDeg) {
        float newC = cosf(angleDeg * DEG2RAD);
        float newS = sinf(angleDeg * DEG2RAD);

        // If buffer is full, subtract the value being overwritten.
        if (filled) {
            sumC -= cosArray[count];
            sumS -= sinArray[count];
        }

        // Insert new sample and add to running sums.
        cosArray[count] = newC;
        sinArray[count] = newS;
        sumC += newC;
        sumS += newS;

        // Advance index
        count++;
        if (count >= sizeOfDataArray) {
            count  = 0;
            filled = true;
        }

        uint16_t n = filled ? sizeOfDataArray : count;
        if (n == 0) return 0.0f;

        // atan2(sumS, sumC) works with summed vectors; scaling by n
        // cancels out in the atan2 result, so we can use sums directly.
        return atan2f(sumS, sumC) * RAD2DEG;
    }

    void reset(void) {
        for (uint16_t i = 0; i < sizeOfDataArray; ++i) {
            cosArray[i] = 0.0f;
            sinArray[i] = 0.0f;
        }
        count  = 0;
        filled = false;
        sumC = 0.0f;
        sumS = 0.0f;
    }
};

#endif // __MOVING_AVERAGE__