#include <Arduino.h>
#include "constants.h"
#include "photo.h"

// Field identifiers
static const uint8_t GREEN_FIELD = 0;
static const uint8_t WHITE_FIELD = 1;

static const uint8_t  SWEEPS_PER_SAMPLE = 4;    // 4 sweeps x 8 channels = 32 readings averaged
static const uint32_t LIVE_INTERVAL_MS  = 80;

static const uint8_t NUMBER_OF_PHOTOS = 8;
static const uint8_t LIMIT_READINGS = 25;
static bool g_liveMode = false;
static uint32_t g_lastLiveMillis = 0;

// Counter to track which side is being measured on the white field
// 1 = Center/Front, 2 = Left, 3 = Right
static uint8_t whiteMeasurementStep = 1;

Phototransistor photo(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

struct SideCalibration {
    bool hasBeenMeasured;
    uint16_t greenAvgChannel[NUMBER_OF_PHOTOS];
    uint16_t whiteAvgChannel[NUMBER_OF_PHOTOS];
    uint16_t threshold;
};

SideCalibration cal_left  = {false, {}, {}};
SideCalibration cal_right = {false, {}, {}};
SideCalibration cal_front = {false, {}, {}};

void measureSidePerChannel(Side side, const char* name, uint8_t fieldType) {
    int numChannels = 0;
    switch (side) {
        case Side::Left:  numChannels = Constants::kPhotoLeftElements;  break;
        case Side::Right: numChannels = Constants::kPhotoRightElements; break;
        case Side::Front: numChannels = Constants::kPhotoFrontElements; break;
    }

    Serial.print(F("Measuring "));
    Serial.print(name);
    Serial.println(F("..."));

    // Accumulate LIMIT_READINGS samples. Each sample itself is an average of
    // SWEEPS_PER_SAMPLE sweeps to reduce noise.
    uint32_t accum[NUMBER_OF_PHOTOS] = {0};

    for (uint8_t sample = 0; sample < LIMIT_READINGS; sample++) {
        uint32_t sampleTotal[NUMBER_OF_PHOTOS] = {0};

        for (uint8_t sweep = 0; sweep < SWEEPS_PER_SAMPLE; sweep++) {
            photo.ReadAllSensors(side);
            for (int ch = 0; ch < numChannels; ch++) {
                sampleTotal[ch] += photo.GetRawReading(side, ch);
            }
        }

        for (int ch = 0; ch < numChannels; ch++) {
            uint16_t sampleAvg = (uint16_t)(sampleTotal[ch] / SWEEPS_PER_SAMPLE);
            accum[ch] += sampleAvg;
        }

        // small visual progress indicator
        Serial.print(F("."));
        delay(20);
    }

    Serial.println();

    // Store final averages into the appropriate SideCalibration struct
    SideCalibration* cal = nullptr;
    switch (side) {
        case Side::Left:  cal = &cal_left;  break;
        case Side::Right: cal = &cal_right; break;
        case Side::Front: cal = &cal_front; break;
    }

    if (cal) {
        for (int ch = 0; ch < numChannels; ch++) {
            uint16_t avg = (uint16_t)(accum[ch] / LIMIT_READINGS);
            Serial.print(F("  Channel "));
            Serial.print(ch);
            Serial.print(F(": avg = "));
            Serial.println(avg);
            if (fieldType == WHITE_FIELD) {
                cal->whiteAvgChannel[ch] = avg;
            } else {
                cal->greenAvgChannel[ch] = avg;
            }
        }
        cal->hasBeenMeasured = true;
    }
}

void printLive(Side side, const char* name) {
    int nch = 0;
    switch (side) {
        case Side::Left:  nch = Constants::kPhotoLeftElements;  break;
        case Side::Right: nch = Constants::kPhotoRightElements; break;
        case Side::Front: nch = Constants::kPhotoFrontElements; break;
    }

    photo.ReadAllSensors(side);
    Serial.print(F("  ["));
    Serial.print(name);
    Serial.print(F("] ch: "));
    for (int ch = 0; ch < nch; ch++) {
        Serial.print(photo.GetRawReading(side, ch));
        if (ch < nch - 1) Serial.print(F(" | "));
    }
    Serial.println();
}

// Read a single non-whitespace byte from Serial if available.
static char readSerialKey() {
    if (!Serial) return '\0';
    if (!Serial.available()) return '\0';

    int c = Serial.read();
    // skip common line endings and spaces
    while (c != -1 && (c == '\r' || c == '\n' || c == ' ' || c == '\t')) {
        if (!Serial.available()) return '\0';
        c = Serial.read();
    }
    if (c == -1) return '\0';
    return static_cast<char>(c);
}

// Handle a key press by passing the character directly
char keyPressed(char c) {
    // If no character was passed, try reading from Serial
    if (c == '\0') c = readSerialKey();
    if (c == '\0') return '\0';

    switch (c)
    {
    case 'c':
        printLive(Side::Front, "FRONT");
        break;
    case 'l':
        printLive(Side::Left, "LEFT ");
        break;
    case 'r':
        printLive(Side::Right, "RIGHT");
        break;
    default:
        Serial.print(F("[photosCalibration2] Unknown key: "));
        Serial.println(c);
        break;
    }

    return c;
}

// Print calibration arrays and threshold constants similar to photosCalibration.cpp
void printCalibrationSummary() {
    // Helper to compute mean of an array for a side
    auto meanOf = [](uint16_t* arr, int count) -> uint32_t {
        uint32_t t = 0;
        for (int i = 0; i < count; i++) t += arr[i];
        return (count > 0) ? (t / count) : 0;
    };

    // Print arrays for a side
    auto printArraysFor = [&](const char* name, SideCalibration& cal, int nch) {
        Serial.print(F("greenAvgChannel "));
        Serial.print(name);
        Serial.print(F(" = {"));
        for (int i = 0; i < nch; i++) {
            Serial.print(cal.greenAvgChannel[i]);
            if (i < nch - 1) Serial.print(F(", "));
        }
        Serial.println(F("};"));

        Serial.print(F("whiteAvgChannel "));
        Serial.print(name);
        Serial.print(F(" = {"));
        for (int i = 0; i < nch; i++) {
            Serial.print(cal.whiteAvgChannel[i]);
            if (i < nch - 1) Serial.print(F(", "));
        }
        Serial.println(F("};"));

        uint32_t meanG = meanOf(cal.greenAvgChannel, nch);
        uint32_t meanW = meanOf(cal.whiteAvgChannel, nch);
        cal.threshold = (uint16_t)((meanG + meanW) / 2);
    };

    int nchLeft  = Constants::kPhotoLeftElements;
    int nchRight = Constants::kPhotoRightElements;
    int nchFront = Constants::kPhotoFrontElements;

    printArraysFor("LEFT ", cal_left, nchLeft);
    printArraysFor("RIGHT", cal_right, nchRight);
    printArraysFor("FRONT", cal_front, nchFront);

    Serial.println();
    Serial.println(F("--------------------------------"));
    Serial.print(F("const int kPhotoTresholdLeft  = "));
    Serial.print(cal_left.threshold);
    Serial.println(F(";"));

    Serial.print(F("const int kPhotoTresholdRight = "));
    Serial.print(cal_right.threshold);
    Serial.println(F(";"));

    Serial.print(F("const int kPhotoTresholdFront = "));
    Serial.print(cal_front.threshold);
    Serial.println(F(";"));

    Serial.println();
    Serial.println(F("============================================"));
}

void setup() {
    Serial.begin(115200);
    photo.Initialize();

    Serial.println(F("=== PHOTO THRESHOLD CALIBRATION 2 ==="));
    Serial.println(F("Robot must be on the GREEN field."));

    // Initial measurements assume robot is on the GREEN field
    measureSidePerChannel(Side::Left, "LEFT ", GREEN_FIELD);
    measureSidePerChannel(Side::Right, "RIGHT", GREEN_FIELD);
    measureSidePerChannel(Side::Front, "FRONT", GREEN_FIELD);

    // Updated instructions
    Serial.println(F("Green calibration complete."));
    Serial.println(F("Commands: 'v' = Continuous Live | 'm' = Step through WHITE field measurements"));
    Serial.println(F("Single Prints: 'c' = Center | 'l' = Left | 'r' = Right"));
    Serial.println(F("\n--> Place CENTER on WHITE field and press 'm' to begin."));
}

void loop() {
    // Non-blocking: read any serial key and act on it.
    char c = readSerialKey();
    if (c != '\0') {
        if (c == 'v') {  // 'v' toggles continuous viewing mode
            g_liveMode = !g_liveMode;
            Serial.print(F("Live mode: "));
            Serial.println(g_liveMode ? F("ON") : F("OFF"));
        } else if (c == 'm') {
            // State machine for measuring the white field
            if (whiteMeasurementStep == 1) {
                Serial.println(F("Measuring FRONT on WHITE field:"));
                measureSidePerChannel(Side::Front, "FRONT", WHITE_FIELD);
                whiteMeasurementStep = 2; // Advance to Left
                Serial.println(F("--> Front complete. Place LEFT on WHITE field and press 'm'."));
            } 
            else if (whiteMeasurementStep == 2) {
                Serial.println(F("Measuring LEFT on WHITE field:"));
                measureSidePerChannel(Side::Left, "LEFT ", WHITE_FIELD);
                whiteMeasurementStep = 3; // Advance to Right
                Serial.println(F("--> Left complete. Place RIGHT on WHITE field and press 'm'."));
            } 
            else if (whiteMeasurementStep == 3) {
                Serial.println(F("Measuring RIGHT on WHITE field:"));
                measureSidePerChannel(Side::Right, "RIGHT", WHITE_FIELD);
                
                Serial.println(F("White-field measurements complete. Generating summary..."));
                printCalibrationSummary();
                
                // Reset step back to 1 in case the user wants to restart the white calibration
                whiteMeasurementStep = 1; 
                Serial.println(F("--> Press 'm' again if you need to restart WHITE field measurements."));
            }
        } else {
            // Feed through to the switch statement cases for c, l, r
            keyPressed(c);
        }
    }

    // If live mode is enabled, periodically print live readings for front, left, right
    if (g_liveMode) {
        uint32_t now = millis();
        if (now - g_lastLiveMillis >= LIVE_INTERVAL_MS) {
            g_lastLiveMillis = now;
            printLive(Side::Front, "FRONT");
            printLive(Side::Left, "LEFT ");
            printLive(Side::Right, "RIGHT");
        }
    }
}