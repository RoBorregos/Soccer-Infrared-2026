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

    static constexpr float DEG2RAD = M_PI / 180.0f;
    static constexpr float RAD2DEG = 180.0f / M_PI;

public:
    CircularMovingAverage(uint16_t num_array) {
        cosArray        = new float[num_array]();
        sinArray        = new float[num_array]();
        sizeOfDataArray = num_array;
        count           = 0;
        filled          = false;
    }

    // Returns the circular mean in degrees (-180 to +180).
    float updateData(float angleDeg) {
        cosArray[count] = cosf(angleDeg * DEG2RAD);
        sinArray[count] = sinf(angleDeg * DEG2RAD);
        count++;
        if (count >= sizeOfDataArray) {
            count  = 0;
            filled = true;
        }
        uint16_t n = filled ? sizeOfDataArray : count;
        float sumC = 0.0f, sumS = 0.0f;
        for (uint16_t i = 0; i < n; ++i) {
            sumC += cosArray[i];
            sumS += sinArray[i];
        }
        return atan2f(sumS, sumC) * RAD2DEG;
    }

    void reset(void) {
        for (uint16_t i = 0; i < sizeOfDataArray; ++i) {
            cosArray[i] = 0.0f;
            sinArray[i] = 0.0f;
        }
        count  = 0;
        filled = false;
    }
};

#endif // __MOVING_AVERAGE__