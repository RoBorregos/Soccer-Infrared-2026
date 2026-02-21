#include <Arduino.h>
#include "IRRing.h"
#include "constants.h"

IRRing irring;

void setup() {
    Serial.begin(115200);
    unsigned long currentTime = millis();
    irring.init(&currentTime);
    irring.SetOffset(0.0);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop(){
    irring.UpdateData();
    double ballAngle = irring.GetAngle(1.0, 1.0, 1.0);
    Serial.print("Ball angle: ");
    Serial.println(ballAngle);
}