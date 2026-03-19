#include <Arduino.h>
#include "goodSerializer.h"

GoodSerializer serializerTest;

void setup(){
    serializerTest.begin();
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
    serializerTest.processLoop();
    
    // Blink the built-in LED to show the loop is running and non-blocking
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink >= 1000) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
    }
}