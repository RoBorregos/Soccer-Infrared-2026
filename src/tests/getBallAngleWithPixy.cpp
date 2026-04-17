#include "robot.h"
#include "Pixy2I2C.h"

Robot followBall;
Pixy2I2C pixy;

uint16_t xComponent = 0;
uint16_t yComponent = 0;
float angle = 0.0;
unsigned long time = 0;

#define SIG_ORANGE_BALL  1

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Pixy2 - Ball Detection Test");
    Serial.println("====================================");

    int result = pixy.init();
    if (result == 0) {
        Serial.println("[SUCCESS] Pixy2 connected!");
    } else {
        Serial.println("[FAIL] Pixy2 not found. Check wiring, PixyMon I2C interface setting, power, and shared ground.");
        while (true); 
    }
}

void loop() {
    unsigned long now = millis();
    if (now - time >= 100) {
        time = now;
    pixy.ccc.getBlocks();
    bool found = false;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == SIG_ORANGE_BALL) {
            found = true;
            Serial.print("BALL| x= ");
            xComponent = pixy.ccc.blocks[i].m_x;
            yComponent = pixy.ccc.blocks[i].m_y;
            angle = (atan2(yComponent, xComponent) * 180 / PI)-45;
            Serial.print(xComponent);
            Serial.print("  y= ");
            Serial.print(yComponent);
            Serial.print("  angle= ");
            Serial.println(angle);
        }
    }

    if (!found) {
        Serial.println("No subject detected");
        } 
    }
}
