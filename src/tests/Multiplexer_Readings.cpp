#include <Arduino.h>
#include "photo.h"
#include "constants.h"

Phototransistor photos(
    Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1,
    Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2,
    Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3);

void setup() {
    Serial.begin(115200);
    photos.Initialize();

    delay(1000);
    Serial.println("Starting photo debug test...");
}

void loop() {
    photos.PhotoDebug();
    Serial.println();
    delay(350);
}

