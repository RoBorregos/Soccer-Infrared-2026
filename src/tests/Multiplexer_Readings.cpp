#include <Arduino.h>
#include "photo.h"
#include "constants.h"

Phototransistor photos(
    Constants::kPhotoLeftSignalPin, Constants::kPhotoLeftMuxS0Pin, Constants::kPhotoLeftMuxS1Pin, Constants::kPhotoLeftMuxS2Pin,
    Constants::kPhotoRightSignalPin, Constants::kPhotoRightMuxS0Pin, Constants::kPhotoRightMuxS1Pin, Constants::kPhotoRightMuxS2Pin,
    Constants::kPhotoFrontSignalPin, Constants::kPhotoFrontMuxS0Pin, Constants::kPhotoFrontMuxS1Pin, Constants::kPhotoFrontMuxS2Pin);

constexpr unsigned long kLateralMuxSwitchIntervalMs = 100;
unsigned long lastLateralMuxSwitchMs = 0;

void printPhotoPinMap() {
    Serial.println("Photo pin map:");

    Serial.print("  LED enable: ");
    Serial.println(Constants::kPhotoLedEnablePin);

    Serial.print("  Left signal/select: ");
    Serial.print(Constants::kPhotoLeftSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoLeftMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoLeftMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoLeftMuxS2Pin);

    Serial.print("  Right signal/select: ");
    Serial.print(Constants::kPhotoRightSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoRightMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoRightMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoRightMuxS2Pin);

    Serial.print("  Front signal/select: ");
    Serial.print(Constants::kPhotoFrontSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoFrontMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoFrontMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoFrontMuxS2Pin);
}

void updateLateralMuxSelection(unsigned long nowMs) {
    if (lastLateralMuxSwitchMs == 0 ||
        (nowMs - lastLateralMuxSwitchMs) >= kLateralMuxSwitchIntervalMs) {
        photos.AdvanceLateralMuxCycle();
        lastLateralMuxSwitchMs = nowMs;
    }
}

void setup() {
    Serial.begin(115200);
#if defined(CORE_TEENSY)
    analogReadResolution(12);
    analogReadAveraging(2);
#endif
    photos.Initialize();
    photos.SetAllMargins(Constants::kPhotoMargins);
    photos.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);
    photos.SetAlternatingLateralMuxEnabled(true);
    photos.AdvanceLateralMuxCycle();
    lastLateralMuxSwitchMs = millis();
    printPhotoPinMap();
    Serial.println("Starting photo debug test...");
}

void loop() {
    updateLateralMuxSelection(millis());
    photos.PhotoDebug();
    if (photos.HasLineOnSide(Side::Left)) {
        Serial.println("LINE on LEFT");
    }
    if (photos.HasLineOnSide(Side::Right)) {
        Serial.println("LINE on RIGHT");
    }
    if (photos.HasLineOnSide(Side::Front)) {
        Serial.println("LINE on FRONT");
    }
    Serial.println();
    delay(120);
}
