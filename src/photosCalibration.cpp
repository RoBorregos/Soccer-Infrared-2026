#include <Arduino.h>
#include "constants.h"
#include "photo.h"

//  1. On boot, robot automatically samples GREEN field
//  2. Serial prompts you to place side on white line -> press any key when ready -> sample white
//
//  LIVE READINGS:
//  At any prompt, send 'l' to toggle a live stream of all channels on the current mux

static const uint8_t  SWEEPS_PER_SAMPLE = 4;    // 4 sweeps x 8 channels = 32 readings averaged
static const uint32_t LIVE_INTERVAL_MS  = 80;

Phototransistor photo(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3
);

struct SideCalibration {
    uint16_t green_avg;
    uint16_t white_avg;
    uint16_t threshold;
    bool     green_done;
    bool     white_done;
};

SideCalibration cal_left  = {0, 0, 0, false, false};
SideCalibration cal_right = {0, 0, 0, false, false};
SideCalibration cal_front = {0, 0, 0, false, false};

// --------------------------------------------------------

uint16_t sampleSurface(Side side) {
    int numChannels = 0;
    switch (side) {
        case Side::Left:  numChannels = Constants::kPhotoLeftElements;  break;
        case Side::Right: numChannels = Constants::kPhotoRightElements; break;
        case Side::Front: numChannels = Constants::kPhotoFrontElements; break;
    }

    uint32_t total = 0;
    uint16_t count = 0;

    for (uint8_t sweep = 0; sweep < SWEEPS_PER_SAMPLE; sweep++) {
        photo.ReadAllSensors(side);
        for (int ch = 0; ch < numChannels; ch++) {
            total += photo.GetRawReading(side, ch);
            count++;
        }
    }
    return (uint16_t)(total / count);
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

char keyPressed(Side side, const char* name) {
    bool live_on = false;
    uint32_t last_live = 0;

    Serial.println(F("  -> Press any key to sample. Press 'l' to toggle live readings."));

    while (true) {
        if (live_on && (millis() - last_live >= LIVE_INTERVAL_MS)) {
            printLive(side, name);
            last_live = millis();
        }

        if (Serial.available()) {
            while (Serial.available() && Serial.peek() <= ' ') Serial.read();
            if (!Serial.available()) continue;

            char key = Serial.read();
            if (key == 'l') {
                live_on = !live_on;
                Serial.println(live_on ? F("  [live ON]") : F("  [live OFF]"));
            } else {
                return key;
            }
        }
    }
}

uint16_t sampleAndPrint(Side side, const char* name, const __FlashStringHelper* surface) {
    Serial.print(F("Calibrating "));
    Serial.print(surface);
    Serial.print(F(" on "));
    Serial.print(name);
    Serial.print(F("..."));

    uint16_t val = sampleSurface(side);
    Serial.print(F(" avg = "));
    Serial.println(val);
    return val;
}

void calibrateSide(Side side, SideCalibration& cal, const char* name) {
    Serial.println();
    Serial.print(F("-----"));
    Serial.print(name);
    Serial.println(F(" SIDE -----"));
    Serial.print(F("  Green avg (sampled at boot): "));
    Serial.println(cal.green_avg);
    Serial.print(F("  Place "));
    Serial.print(name);
    Serial.println(F(" sensors on the WHITE line."));

    keyPressed(side, name);

    cal.white_avg  = sampleAndPrint(side, name, F("WHITE"));
    cal.threshold  = (cal.green_avg + cal.white_avg) / 2;
    cal.white_done = true;

    Serial.print(F("  Threshold (midpoint): "));
    Serial.println(cal.threshold);
}

void printSummary() {
    Serial.println();
    Serial.println(F("============================================"));
    Serial.println(F("CALIBRATION COMPLETE"));
    Serial.println();
    Serial.println(F("  Side   | Green avg | White avg | Threshold"));
    Serial.println(F("  -------|-----------|-----------|----------"));

    auto printRow = [](const char* name, SideCalibration& c) {
        Serial.print(F("  "));
        Serial.print(name);
        Serial.print(F("  |   "));
        Serial.print(c.green_avg);
        Serial.print(F("     |   "));
        Serial.print(c.white_avg);
        Serial.print(F("     |   "));
        Serial.println(c.threshold);
    };

    printRow("LEFT ", cal_left);
    printRow("RIGHT", cal_right);
    printRow("FRONT", cal_front);

    Serial.println();
    Serial.println(F("--------------------------------"));
    Serial.print(F("const int kPhotoTresholdLeft  = ")); Serial.print(cal_left.threshold);  Serial.println(F(";"));
    Serial.print(F("const int kPhotoTresholdRight = ")); Serial.print(cal_right.threshold); Serial.println(F(";"));
    Serial.print(F("const int kPhotoTresholdFront = ")); Serial.print(cal_front.threshold); Serial.println(F(";"));
    Serial.println();
    Serial.println(F("============================================"));
}

void setup() {
    Serial.begin(115200);
    photo.Initialize();

    Serial.println(F("=== PHOTO THRESHOLD CALIBRATION ==="));
    Serial.println(F("Robot must be on the GREEN field. Sampling green..."));
    Serial.println();

    cal_left.green_avg  = sampleAndPrint(Side::Left,  "LEFT ",  F("GREEN"));
    cal_right.green_avg = sampleAndPrint(Side::Right, "RIGHT",  F("GREEN"));
    cal_front.green_avg = sampleAndPrint(Side::Front, "FRONT",  F("GREEN"));

    cal_left.green_done  = true;
    cal_right.green_done = true;
    cal_front.green_done = true;

    Serial.println();
    Serial.println(F("Green calibrated. Starting white calibration per side."));
    Serial.println(F("Press 'l' at any prompt to see live channel readings."));

    calibrateSide(Side::Front, cal_front, "FRONT");
    calibrateSide(Side::Left,  cal_left,  "LEFT ");
    calibrateSide(Side::Right, cal_right, "RIGHT");

    printSummary();
}

void loop() {
    // all done only once so theres no need to loop, press reset on arduino to run again
}