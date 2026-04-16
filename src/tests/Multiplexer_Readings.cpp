#include <Arduino.h>
#include "photo.h"
#include "constants.h"

Phototransistor photos(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3);

void printPhotoPinMap() {
    Serial.println("Photo pin map:");

    Serial.print("  LED enable: ");
    Serial.println(Constants::kPhotoLedEnablePin);

    Serial.print("  Bank 1 signal/select: ");
    Serial.print(Constants::kSignalPin1);
    Serial.print(" / ");
    Serial.print(Constants::kMUXPin1_1);
    Serial.print(", ");
    Serial.print(Constants::kMUXPin2_1);
    Serial.print(", ");
    Serial.println(Constants::kMUXPin3_1);

    Serial.print("  Bank 2 signal/select: ");
    Serial.print(Constants::kSignalPin2);
    Serial.print(" / ");
    Serial.print(Constants::kMUXPin1_2);
    Serial.print(", ");
    Serial.print(Constants::kMUXPin2_2);
    Serial.print(", ");
    Serial.println(Constants::kMUXPin3_2);

    Serial.print("  Bank 3 signal/select: ");
    Serial.print(Constants::kSignalPin3);
    Serial.print(" / ");
    Serial.print(Constants::kMUXPin1_3);
    Serial.print(", ");
    Serial.print(Constants::kMUXPin2_3);
    Serial.print(", ");
    Serial.println(Constants::kMUXPin3_3);
}

void setup() {
    Serial.begin(115200);
#if defined(CORE_TEENSY)
    analogReadResolution(12);
    analogReadAveraging(2);
#endif
    photos.Initialize();
    photos.SetAllMargins(Constants::kPhotoMargins);

    delay(1000);
    photos.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);
    printPhotoPinMap();
    Serial.println("Starting photo debug test...");
}

void loop() {
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

