#ifndef PHOTO_H
#define PHOTO_H

#include <Arduino.h>
#include "constants.h"
#include "Multiplexer.h"

enum class Side
{
    Left,
    Right,
    Front
};

struct PhotoData
{
    bool is_on_line;
    int correction_degree;
};

class Phototransistor
{
private:
    struct SideData
    {
        Multiplexer *mux;
        uint16_t *readings;
        uint16_t *baseline;
        uint16_t *margins;
        const uint16_t *fixed_thresholds;
        uint8_t elements;
        uint8_t *confirmation_counter;
        bool *baseline_captured;
        int correction_degree;
    };

    Multiplexer left_mux_;
    Multiplexer right_mux_;
    Multiplexer front_mux_;

    uint16_t photo_left_[Constants::kPhotoLeftElements];
    uint16_t photo_right_[Constants::kPhotoRightElements];
    uint16_t photo_front_[Constants::kPhotoFrontElements];
    uint16_t left_baseline_[Constants::kPhotoLeftElements] = {0};
    uint16_t right_baseline_[Constants::kPhotoRightElements] = {0};
    uint16_t front_baseline_[Constants::kPhotoFrontElements] = {0};
    uint16_t left_margins_[Constants::kPhotoLeftElements] = {0};
    uint16_t right_margins_[Constants::kPhotoRightElements] = {0};
    uint16_t front_margins_[Constants::kPhotoFrontElements] = {0};
    bool left_baseline_captured_ = false;
    bool right_baseline_captured_ = false;
    bool front_baseline_captured_ = false;

    uint8_t left_line_confirmations_ = 0;
    uint8_t right_line_confirmations_ = 0;
    uint8_t front_line_confirmations_ = 0;

    uint8_t required_line_confirmations_ = 2;
    uint16_t threshold_padding_ = 5;

    SideData GetSideData(Side side);
    SideData GetSideData(Side side) const;
    uint16_t GetActiveThreshold(const SideData &side_data, uint8_t channel) const;
    bool HasLineReading(const SideData &side_data) const;
    void UpdateConfirmationCounter(const SideData &side_data, bool detected);

public:
    Phototransistor(uint8_t sig_left, uint8_t s0_l, uint8_t s1_l, uint8_t s2_l,
                    uint8_t sig_right, uint8_t s0_r, uint8_t s1_r, uint8_t s2_r,
                    uint8_t sig_front, uint8_t s0_f, uint8_t s1_f, uint8_t s2_f);

    uint16_t GetRawReading(Side side, uint8_t channel);
    uint16_t GetThreshold(Side side, uint8_t channel) const;
    uint16_t GetBaselineReading(Side side, uint8_t channel) const;

    void Initialize();
    void ReadAllSensors(Side side);
    void CaptureSideBaseline(Side side, uint8_t samples, uint16_t delay_ms);
    void CaptureBaseline(uint8_t samples, uint16_t delay_ms);
    void SetMargins(Side side, const uint16_t *margins, uint8_t num_elements);
    void SetRequiredConfirmations(uint8_t confirmations);
    void SetThresholdPadding(uint16_t padding);
    PhotoData CheckPhotosOnField(Side side);

    void ReadMuxSide(Multiplexer &mux, uint16_t *target_array, int element_count);
};

#endif // PHOTO_H