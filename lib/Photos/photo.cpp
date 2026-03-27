#include "photo.h"

Phototransistor::Phototransistor(uint8_t signal_left, uint8_t s0_l, uint8_t s1_l, uint8_t s2_l,
             uint8_t signal_right, uint8_t s0_r, uint8_t s1_r, uint8_t s2_r,
             uint8_t signal_front, uint8_t s0_f, uint8_t s1_f, uint8_t s2_f)
    : left_mux_(signal_left, s0_l, s1_l, s2_l),
      right_mux_(signal_right, s0_r, s1_r, s2_r),
      front_mux_(signal_front, s0_f, s1_f, s2_f) {}

void Phototransistor::Initialize()
{
    left_mux_.InitializeMultiplexer();
    right_mux_.InitializeMultiplexer();
    front_mux_.InitializeMultiplexer();
}

// Helper function to read a whole side
void Phototransistor::ReadMuxSide(Multiplexer &mux, uint16_t *target_array, int num_elements)
{
    for (int i = 0; i < num_elements; i++)
    {
        target_array[i] = mux.readChannel(i);
    }
}

// Read all sensors in one go and then process the data using the helper function
void Phototransistor::ReadAllSensors(Side side)
{
    switch (side)
    {
    case Side::Left:
        ReadMuxSide(left_mux_, photo_left_, Constants::kPhotoLeftElements);
        break;

    case Side::Right:
        ReadMuxSide(right_mux_, photo_right_, Constants::kPhotoRightElements);
        break;

    case Side::Front:
        ReadMuxSide(front_mux_, photo_front_, Constants::kPhotoFrontElements);
        break;
    }
}

PhotoData Phototransistor::CheckPhotosOnField(Side side)
{
    // Initialize the struct: default to no line and 0 degrees
    
    PhotoData data = {false, 0};

    ReadAllSensors(side);

    uint16_t *current_array;
    int threshold;
    int elements;

    switch (side)
    {
    case Side::Left:
        current_array = photo_left_;
        threshold = Constants::kPhotoTresholdLeft;
        elements = Constants::kPhotoLeftElements;
        data.correction_degree = -90;
        break;
    case Side::Right:
        current_array = photo_right_;
        threshold = Constants::kPhotoTresholdRight;
        elements = Constants::kPhotoRightElements;
        data.correction_degree = 90;
        break;
    case Side::Front:
        current_array = photo_front_;
        threshold = Constants::kPhotoTresholdFront;
        elements = Constants::kPhotoFrontElements;
        data.correction_degree = 180;
        break;
    }

    for (int i = 0; i < elements; i++)
    {
        // If any sensor breaks the threshold, we hit the line.
        if (current_array[i] > threshold)
        {
            data.is_on_line = true;
            break;
        }
    }

    return data;
}

uint16_t Phototransistor::GetRawReading(Side side, uint8_t channel)
{
    switch (side)
    {
    case Side::Left:
        if (channel >= Constants::kPhotoLeftElements)
        {
            return 0;
        }
        return photo_left_[channel];
    case Side::Right:
        if (channel >= Constants::kPhotoRightElements)
        {
            return 0;
        }
        return photo_right_[channel];
    case Side::Front:
        if (channel >= Constants::kPhotoFrontElements)
        {
            return 0;
        }
        return photo_front_[channel];
    default:
        return 0;
    }
}
