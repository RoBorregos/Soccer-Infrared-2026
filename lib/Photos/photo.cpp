#include "photo.h"

Phototransistor::Phototransistor(uint8_t signal_left, uint8_t s0_l, uint8_t s1_l, uint8_t s2_l,
             uint8_t signal_right, uint8_t s0_r, uint8_t s1_r, uint8_t s2_r,
             uint8_t signal_front, uint8_t s0_f, uint8_t s1_f, uint8_t s2_f)
    : left_mux_(signal_left, s0_l, s1_l, s2_l),
      right_mux_(signal_right, s0_r, s1_r, s2_r),
      front_mux_(signal_front, s0_f, s1_f, s2_f) {}

// int Phototransistor::GetEscapeAngle(const PhotoData &front, const PhotoData &left, const PhotoData &right)
int Phototransistor::GetEscapeAngle(const PhotoData &front)
{
    if (front.is_on_line)
    {
        return front.correction_degree;
    }

    // if (left.is_on_line)
    // {
    //     return left.correction_degree;
    // }

    // if (right.is_on_line)
    // {
    //     return right.correction_degree;
    // }

    return -1;
}

Phototransistor::SideData Phototransistor::GetSideData(Side side) // A simple helper to get data from x side
{
    switch (side)
    {
    case Side::Left:
        return {
            &left_mux_,
            photo_left_,
            left_baseline_,
            left_margins_,
            Constants::kPhotoLeftThresholds,
            Constants::kPhotoLeftElements,
            &left_baseline_captured_,
            -90
        };
    case Side::Right:
        return {
            &right_mux_,
            photo_right_,
            right_baseline_,
            right_margins_,
            Constants::kPhotoRightThresholds,
            Constants::kPhotoRightElements,
            &right_baseline_captured_,
            90
        };
    case Side::Front:
    default:
        return {
            &front_mux_,
            photo_front_,
            front_baseline_,
            front_margins_,
            Constants::kPhotoFrontThresholds,
            Constants::kPhotoFrontElements,
            &front_baseline_captured_,
            180
        };
    }
}

Phototransistor::SideData Phototransistor::GetSideData(Side side) const
{
    return const_cast<Phototransistor *>(this)->GetSideData(side);
}

uint16_t Phototransistor::GetActiveThreshold(const SideData &side_data, uint8_t channel) const
{
    uint16_t active_threshold = side_data.fixed_thresholds[channel];
    if (*side_data.baseline_captured)
    {
        const uint16_t baseline_threshold = side_data.baseline[channel] + side_data.margins[channel];
        const uint16_t fixed_threshold = side_data.fixed_thresholds[channel] + threshold_padding_;
        active_threshold = max(baseline_threshold, fixed_threshold);
    }

    return active_threshold;
}

bool Phototransistor::HasLineReading(const SideData &side_data) const // Main logic to determine if we have line on a side
{
    for (uint8_t channel = 0; channel < side_data.elements; channel++)
    {
        if (side_data.readings[channel] > GetActiveThreshold(side_data, channel))
        {
            return true;
        }
    }

    return false;
}

void Phototransistor::Initialize()
{
    left_mux_.InitializeMultiplexer();
    right_mux_.InitializeMultiplexer();
    front_mux_.InitializeMultiplexer();
}

void Phototransistor::ReadMuxSide(Multiplexer &mux, uint16_t *target_array, int element_count) // Singular channel reading
{
    for (int i = 0; i < element_count; i++)
    {
        target_array[i] = mux.readChannel(i);
    }
}

void Phototransistor::ReadAllSensors(Side side) // Complete channel reading
{
    SideData side_data = GetSideData(side);
    ReadMuxSide(*side_data.mux, side_data.readings, side_data.elements);
}

void Phototransistor::CaptureSideBaseline(Side side, uint8_t samples, uint16_t delay_ms) // Captures baseline for a specific side with given samples 
{
    if (samples == 0)
    {
        return;
    }

    SideData side_data = GetSideData(side);
    uint32_t sums[Constants::kPhotoRightElements] = {0};

    for (uint8_t sample = 0; sample < samples; sample++)
    {
        ReadAllSensors(side);
        for (uint8_t channel = 0; channel < side_data.elements; channel++)
        {
            sums[channel] += side_data.readings[channel];
        }
        delay(delay_ms);
    }

    for (uint8_t channel = 0; channel < side_data.elements; channel++)
    {
        side_data.baseline[channel] = sums[channel] / samples;
    }

    *side_data.baseline_captured = true;
}

void Phototransistor::CaptureBaseline(uint8_t samples, uint16_t delay_ms) // Captures baseline for all sides
{
    CaptureSideBaseline(Side::Left, samples, delay_ms);
    CaptureSideBaseline(Side::Right, samples, delay_ms);
    CaptureSideBaseline(Side::Front, samples, delay_ms);
}

void Phototransistor::SetMargins(Side side, const uint16_t *margins, uint8_t num_elements) 
{
    SideData side_data = GetSideData(side);
    const uint8_t limit = num_elements < side_data.elements ? num_elements : side_data.elements;

    for (uint8_t channel = 0; channel < limit; channel++)
    {
        side_data.margins[channel] = margins[channel];
    }
}

void Phototransistor::SetThresholdPadding(uint16_t padding)
{
    threshold_padding_ = padding;
}

PhotoData Phototransistor::CheckPhotosOnField(Side side) // Main function to check if we have line on a side and return correction degree if we do
{
    ReadAllSensors(side);

    SideData side_data = GetSideData(side);
    return {
        HasLineReading(side_data),
        side_data.correction_degree
    };
}

uint16_t Phototransistor::GetThreshold(Side side, uint8_t channel) const
{
    SideData side_data = GetSideData(side);
    if (channel >= side_data.elements)
    {
        return 0;
    }

    return GetActiveThreshold(side_data, channel);
}

uint16_t Phototransistor::GetBaselineReading(Side side, uint8_t channel) const
{
    SideData side_data = GetSideData(side);
    if (channel >= side_data.elements)
    {
        return 0;
    }

    return side_data.baseline[channel];
}
