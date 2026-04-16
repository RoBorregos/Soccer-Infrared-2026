#ifndef PHOTO_H
#define PHOTO_H

#include <Arduino.h>
#include "constants.h"
#include "multiplexer.h"

enum class Side
{
    Left,
    Right,
    Front
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
        bool *baseline_captured;
        int correction_degree;
    };

    Multiplexer left_mux_;
    Multiplexer right_mux_;
    Multiplexer front_mux_;

    uint16_t photo_left_[Constants::kPhotoElements];
    uint16_t photo_right_[Constants::kPhotoElements];
    uint16_t photo_front_[Constants::kPhotoElements];
    uint16_t left_baseline_[Constants::kPhotoElements] = {0};
    uint16_t right_baseline_[Constants::kPhotoElements] = {0};
    uint16_t front_baseline_[Constants::kPhotoElements] = {0};
    uint16_t left_margins_[Constants::kPhotoElements] = {0};
    uint16_t right_margins_[Constants::kPhotoElements] = {0};
    uint16_t front_margins_[Constants::kPhotoElements] = {0};
    
    bool left_baseline_captured_ = false;
    bool right_baseline_captured_ = false;
    bool front_baseline_captured_ = false;
    SideData GetSideData(Side side);
    bool IsSideEnabled(Side side) const;
    bool HasLineReading(const SideData &side_data) const;
    void ReadMuxChannels(Multiplexer &mux, uint16_t *target_array);

public:
    Phototransistor(uint8_t left_signal_pin, uint8_t left_mux_s0_pin, uint8_t left_mux_s1_pin, uint8_t left_mux_s2_pin,
                    uint8_t right_signal_pin, uint8_t right_mux_s0_pin, uint8_t right_mux_s1_pin, uint8_t right_mux_s2_pin,
                    uint8_t front_signal_pin, uint8_t front_mux_s0_pin, uint8_t front_mux_s1_pin, uint8_t front_mux_s2_pin);

    void Initialize();
    void SetIlluminationEnabled(bool enabled);
    void ReadAllSensors(Side side);
    void CaptureSideBaseline(Side side, uint8_t samples, uint16_t delay_ms);
    void CaptureBaseline(uint8_t samples, uint16_t delay_ms);
    void SetMargins(Side side, const uint16_t *margins);
    void SetAllMargins(const uint16_t margins[3][Constants::kPhotoElements]);
    void PhotoDebug();
    bool HasLineOnSide(Side side);
    int CheckPhotosOnField(); // Returns an escape angle, or -1 when no line is detected
    void ReadMuxSide(Side side, uint16_t *target_array);
};

#endif // PHOTO_H
