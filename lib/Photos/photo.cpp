#include "photo.h"

Phototransistor::Phototransistor(uint8_t left_signal_pin, uint8_t left_mux_s0_pin, uint8_t left_mux_s1_pin, uint8_t left_mux_s2_pin,
             uint8_t right_signal_pin, uint8_t right_mux_s0_pin, uint8_t right_mux_s1_pin, uint8_t right_mux_s2_pin,
             uint8_t front_signal_pin, uint8_t front_mux_s0_pin, uint8_t front_mux_s1_pin, uint8_t front_mux_s2_pin)
    : left_mux_(left_signal_pin, left_mux_s0_pin, left_mux_s1_pin, left_mux_s2_pin),
      right_mux_(right_signal_pin, right_mux_s0_pin, right_mux_s1_pin, right_mux_s2_pin),
      front_mux_(front_signal_pin, front_mux_s0_pin, front_mux_s1_pin, front_mux_s2_pin) {}

Phototransistor::SideData Phototransistor::GetSideData(Side side) // A simple helper to get data from x side
{
    switch (side)
    {
    case Side::Left:
        return SideData{
            &left_mux_,
            photo_left_,
            left_baseline_,
            left_margins_,
            &left_baseline_captured_,
            90
        };
    case Side::Right:
        return SideData{
            &right_mux_,
            photo_right_,
            right_baseline_,
            right_margins_,
            &right_baseline_captured_,
            -90
        };
    case Side::Front:
    default:
        return SideData{
            &front_mux_,
            photo_front_,
            front_baseline_,
            front_margins_,
            &front_baseline_captured_,
            180
        };
    }
}

bool Phototransistor::IsSideEnabled(Side side) const
{
    switch (side)
    {
    case Side::Left:
        return Constants::kPhotoLeftEnabled;
    case Side::Right:
        return Constants::kPhotoRightEnabled;
    case Side::Front:
    default:
        return Constants::kPhotoFrontEnabled;
    }
}

bool Phototransistor::HasLineReading(const SideData &side_data) const // Main logic to determine if we have line on a side
{
    for (uint8_t channel = 0; channel < Constants::kPhotoElements; channel++)
    {
        if (side_data.readings[channel] > (side_data.baseline[channel] + side_data.margins[channel]))
        {
            return true;
        }
    }

    return false;
}

void Phototransistor::ReadMuxChannels(Multiplexer &mux, uint16_t *target_array)
{
    for (uint8_t i = 0; i < Constants::kPhotoElements; i++)
    {
        target_array[i] = mux.readChannel(i);
    }
}

void Phototransistor::ReadMuxSide(Side side, uint16_t *target_array) // Singular channel reading
{
    if (!IsSideEnabled(side))
    {
        for (uint8_t i = 0; i < Constants::kPhotoElements; i++)
        {
            target_array[i] = 0;
        }
        return;
    }

    SideData side_data = GetSideData(side);
    ReadMuxChannels(*side_data.mux, target_array);
}


void Phototransistor::Initialize()
{
    SetIlluminationEnabled(true);

    if (Constants::kPhotoLeftEnabled)
    {
        left_mux_.InitializeMultiplexer();
    }

    if (Constants::kPhotoRightEnabled)
    {
        right_mux_.InitializeMultiplexer();
    }

    if (Constants::kPhotoFrontEnabled)
    {
        front_mux_.InitializeMultiplexer();
    }
}

void Phototransistor::SetIlluminationEnabled(bool enabled)
{
    pinMode(Constants::kPhotoLedEnablePin, OUTPUT);
    digitalWrite(Constants::kPhotoLedEnablePin, enabled ? HIGH : LOW);
}

void Phototransistor::ReadAllSensors(Side side) // Complete channel reading
{
    if (!IsSideEnabled(side))
    {
        return;
    }

    SideData side_data = GetSideData(side);
    ReadMuxChannels(*side_data.mux, side_data.readings);
}

void Phototransistor::CaptureSideBaseline(Side side, uint8_t samples, uint16_t delay_ms) // Captures baseline for a specific side with given samples 
{
    if (samples == 0 || !IsSideEnabled(side))
    {
        return;
    }

    SideData side_data = GetSideData(side);
    uint32_t sums[Constants::kPhotoElements] = {0};

    for (uint8_t sample = 0; sample < samples; sample++)
    {
        ReadAllSensors(side);
        for (uint8_t channel = 0; channel < Constants::kPhotoElements; channel++)
        {
            sums[channel] += side_data.readings[channel];
        }
        delay(delay_ms);
    }

    for (uint8_t channel = 0; channel < Constants::kPhotoElements; channel++)
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

void Phototransistor::SetMargins(Side side, const uint16_t *margins) 
{
    SideData side_data = GetSideData(side);
    for (uint8_t channel = 0; channel < Constants::kPhotoElements; channel++)
    {
        side_data.margins[channel] = margins[channel];
    }
}

void Phototransistor::SetAllMargins(const uint16_t margins[3][Constants::kPhotoElements]) 
{
    SetMargins(Side::Left, margins[0]);
    SetMargins(Side::Right, margins[1]);
    SetMargins(Side::Front, margins[2]);
}

void Phototransistor::PhotoDebug()
{
    uint16_t left_readings[Constants::kPhotoElements] = {0};
    uint16_t right_readings[Constants::kPhotoElements] = {0};
    uint16_t front_readings[Constants::kPhotoElements] = {0};

    ReadMuxSide(Side::Left, left_readings);
    ReadMuxSide(Side::Right, right_readings);
    ReadMuxSide(Side::Front, front_readings);

    Serial.println("Ch\tLeftB\tLeft\tRightB\tRight\tFrontB\tFront");

    for (uint8_t channel = 0; channel < Constants::kPhotoElements; channel++)
    {
        Serial.print(channel);
        Serial.print('\t');
        Serial.print(left_baseline_[channel]);
        Serial.print('\t');
        Serial.print(left_readings[channel]);
        Serial.print('\t');
        Serial.print(right_baseline_[channel]);
        Serial.print('\t');
        Serial.print(right_readings[channel]);
        Serial.print('\t');
        Serial.print(front_baseline_[channel]);
        Serial.print('\t');
        Serial.println(front_readings[channel]);
    }
}

bool Phototransistor::HasLineOnSide(Side side)
{
    if (!IsSideEnabled(side))
    {
        return false;
    }

    ReadAllSensors(side);
    const SideData side_data = GetSideData(side);
    return HasLineReading(side_data);
}

int Phototransistor::CheckPhotosOnField() // Check all sides and return the first escape angle
{
    if (IsSideEnabled(Side::Front))
    {
        if (HasLineOnSide(Side::Front))
        {
            SideData front = GetSideData(Side::Front);
            return front.correction_degree;
        }
    }

    if (IsSideEnabled(Side::Left))
    {
        if (HasLineOnSide(Side::Left))
        {
            SideData left = GetSideData(Side::Left);
            return left.correction_degree;
        }
    }

    if (IsSideEnabled(Side::Right))
    {
        if (HasLineOnSide(Side::Right))
        {
            SideData right = GetSideData(Side::Right);
            return right.correction_degree;
        }
    }

    return -1;
}
