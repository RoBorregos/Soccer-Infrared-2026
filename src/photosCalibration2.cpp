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

Phototransistor photo(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

struct SideCalibration {
    bool hasBeenMeasured;
    uint16_t greenAvgChannel[NUMBER_OF_PHOTOS];
    uint16_t whiteAvgChannel[NUMBER_OF_PHOTOS];
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

// Handle a key press. If `key` is null or empty, try reading from Serial.
char keyPressed(const char* key) {
    char c = '\0';
    if (key && key[0]) c = key[0];
    else c = readSerialKey();

    if (c == '\0') return '\0';

    switch (c)
    {
    case 'l':
        Serial.println(F("[photosCalibration2] 'l' pressed"));
        break;

    case 'm':
        Serial.println(F("[photosCalibration2] 'm' pressed"));
        break;

    default:
        Serial.print(F("[photosCalibration2] Unknown key: "));
        Serial.println(c);
        break;
    }

    return c;
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

    Serial.println(F("Calibration complete. Press 'l' to see live readings."));
}

void loop() {
    // Non-blocking: read any serial key and act on it.
    char c = readSerialKey();
    if (c != '\0') {
        if (c == 'l') {
            g_liveMode = !g_liveMode;
            Serial.print(F("Live mode: "));
            Serial.println(g_liveMode ? F("ON") : F("OFF"));
        } else if (c == 'm') {
            Serial.println(F("Starting measurements (WHITE field):"));
            // center, then left, then right
            measureSidePerChannel(Side::Front, "CENTER", WHITE_FIELD);
            measureSidePerChannel(Side::Left, "LEFT ", WHITE_FIELD);
            measureSidePerChannel(Side::Right, "RIGHT", WHITE_FIELD);
            Serial.println(F("White-field measurements complete."));
        } else {
            // feed through to existing handler for logging/other uses
            keyPressed(&c);
        }
    }

    // If live mode is enabled, periodically print live readings for center,left,right
    if (g_liveMode) {
        uint32_t now = millis();
        if (now - g_lastLiveMillis >= LIVE_INTERVAL_MS) {
            g_lastLiveMillis = now;
            printLive(Side::Front, "CENTER");
            printLive(Side::Left, "LEFT ");
            printLive(Side::Right, "RIGHT");
        }
    }
    
}