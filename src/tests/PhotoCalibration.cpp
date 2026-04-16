#include <Arduino.h>
#include "photo.h"
#include "constants.h"

Phototransistor photos(
    Constants::kPhotoLeftSignalPin, Constants::kPhotoLeftMuxS0Pin, Constants::kPhotoLeftMuxS1Pin, Constants::kPhotoLeftMuxS2Pin,
    Constants::kPhotoRightSignalPin, Constants::kPhotoRightMuxS0Pin, Constants::kPhotoRightMuxS1Pin, Constants::kPhotoRightMuxS2Pin,
    Constants::kPhotoFrontSignalPin, Constants::kPhotoFrontMuxS0Pin, Constants::kPhotoFrontMuxS1Pin, Constants::kPhotoFrontMuxS2Pin);

namespace {
    constexpr uint8_t kChannels = Constants::kPhotoElements;
    constexpr uint8_t kBaselineSamples = 40;
    constexpr unsigned long kReportIntervalMs = 1000;
    constexpr float kSuggestedThresholdRatio = 0.67f;
    constexpr uint16_t kMinimumSuggestedThreshold = 20;
    constexpr uint16_t kMaximumSuggestedThreshold = 150;

    uint16_t baselineLeft[kChannels] = {0};
    uint16_t baselineRight[kChannels] = {0};
    uint16_t baselineFront[kChannels] = {0};
    uint16_t currentLeft[kChannels] = {0};
    uint16_t currentRight[kChannels] = {0};
    uint16_t currentFront[kChannels] = {0};
    uint16_t maxDeltaLeft[kChannels] = {0};
    uint16_t maxDeltaRight[kChannels] = {0};
    uint16_t maxDeltaFront[kChannels] = {0};
    unsigned long lastReportMs = 0;
}

void captureBaselineForSide(Side side, uint16_t* baselineBuffer) {
    uint32_t sums[kChannels] = {0};
    uint16_t readings[kChannels] = {0};

    for (uint8_t sample = 0; sample < kBaselineSamples; sample++) {
        photos.ReadMuxSide(side, readings);
        for (uint8_t channel = 0; channel < kChannels; channel++) {
            sums[channel] += readings[channel];
        }
        delay(Constants::kBaselineDelayMs);
    }

    for (uint8_t channel = 0; channel < kChannels; channel++) {
        baselineBuffer[channel] = static_cast<uint16_t>(sums[channel] / kBaselineSamples);
    }
}

void updateMaxDelta(const uint16_t* baselineBuffer, const uint16_t* currentBuffer, uint16_t* maxDeltaBuffer) {
    for (uint8_t channel = 0; channel < kChannels; channel++) {
        const uint16_t delta =
            currentBuffer[channel] > baselineBuffer[channel]
                ? static_cast<uint16_t>(currentBuffer[channel] - baselineBuffer[channel])
                : 0;

        if (delta > maxDeltaBuffer[channel]) {
            maxDeltaBuffer[channel] = delta;
        }
    }
}

uint16_t suggestedThreshold(uint16_t observedDelta) {
    const uint16_t scaled = static_cast<uint16_t>(observedDelta * kSuggestedThresholdRatio);
    uint16_t suggested = scaled > kMinimumSuggestedThreshold ? scaled : kMinimumSuggestedThreshold;
    if (suggested > kMaximumSuggestedThreshold) {
        suggested = kMaximumSuggestedThreshold;
    }
    return suggested;
}

void printSideTable(const char* label,
                    const uint16_t* baselineBuffer,
                    const uint16_t* currentBuffer,
                    const uint16_t* maxDeltaBuffer) {
    Serial.println(label);
    Serial.println("ch\tbase\tnow\tdelta\tmax\tsuggest");

    for (uint8_t channel = 0; channel < kChannels; channel++) {
        const uint16_t delta =
            currentBuffer[channel] > baselineBuffer[channel]
                ? static_cast<uint16_t>(currentBuffer[channel] - baselineBuffer[channel])
                : 0;

        Serial.print(channel);
        Serial.print('\t');
        Serial.print(baselineBuffer[channel]);
        Serial.print('\t');
        Serial.print(currentBuffer[channel]);
        Serial.print('\t');
        Serial.print(delta);
        Serial.print('\t');
        Serial.print(maxDeltaBuffer[channel]);
        Serial.print('\t');
        Serial.println(suggestedThreshold(maxDeltaBuffer[channel]));
    }

    Serial.println();
}

void printSuggestedMarginArray(const char* label, const uint16_t* maxDeltaBuffer) {
    Serial.print(label);
    Serial.print(": {");
    for (uint8_t channel = 0; channel < kChannels; channel++) {
        Serial.print(suggestedThreshold(maxDeltaBuffer[channel]));
        if (channel + 1 < kChannels) {
            Serial.print(", ");
        }
    }
    Serial.println("}");
}

void setup() {
    Serial.begin(115200);
#if defined(CORE_TEENSY)
    analogReadResolution(12);
    analogReadAveraging(8);
#endif
    photos.Initialize();

    delay(1000);
    Serial.println("Photo calibration starting");
    Serial.println("Keep robot off the line for baseline capture...");
    delay(1500);

    captureBaselineForSide(Side::Left, baselineLeft);
    captureBaselineForSide(Side::Right, baselineRight);
    captureBaselineForSide(Side::Front, baselineFront);

    Serial.println("Baseline captured");
    Serial.println("Now move each side over the line and watch the suggested thresholds grow.");
    Serial.println();
}

void loop() {
    photos.ReadMuxSide(Side::Left, currentLeft);
    photos.ReadMuxSide(Side::Right, currentRight);
    photos.ReadMuxSide(Side::Front, currentFront);

    updateMaxDelta(baselineLeft, currentLeft, maxDeltaLeft);
    updateMaxDelta(baselineRight, currentRight, maxDeltaRight);
    updateMaxDelta(baselineFront, currentFront, maxDeltaFront);

    if (millis() - lastReportMs < kReportIntervalMs) {
        return;
    }

    lastReportMs = millis();

    printSideTable("LEFT", baselineLeft, currentLeft, maxDeltaLeft);
    printSideTable("RIGHT", baselineRight, currentRight, maxDeltaRight);
    printSideTable("FRONT", baselineFront, currentFront, maxDeltaFront);

    Serial.println("Suggested kPhotoMargins rows:");
    printSuggestedMarginArray("left", maxDeltaLeft);
    printSuggestedMarginArray("right", maxDeltaRight);
    printSuggestedMarginArray("front", maxDeltaFront);
    Serial.println();
}
