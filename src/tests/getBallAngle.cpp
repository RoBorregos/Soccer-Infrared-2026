#include <Arduino.h>
#include "IRRing.h"
#include "constants.h"

IRRing irring;

float kBallFollowOffsetBack = 1.0;
float kBallFollowOffsetSide = 1.0;
float kBallFollowOffsetFront = 1.0;

void setup() {
    Serial.begin(115200);
    unsigned long currentTime = millis();
    irring.init(&currentTime);
    irring.SetOffset(0.0);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop(){
    irring.UpdateData();
    double ballAngle = irring.GetAngle(kBallFollowOffsetBack, kBallFollowOffsetSide, kBallFollowOffsetFront);
    double ballStrength = irring.GetStrength();
    Serial.print("Ball angle: ");
    Serial.print(ballAngle);
    Serial.print(" strength: ");
    Serial.println(ballStrength);
}
