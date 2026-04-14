#include <Arduino.h>
#include "BinaryStreamProcessor.h"

BinaryStreamProcessor receiver;

void setup() {
    receiver.begin(); // defaults: Serial 115200, UART 57600
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    receiver.processLoop();

    // Non-blocking blink to show main loop is alive
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink >= 1000) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
    }
}
