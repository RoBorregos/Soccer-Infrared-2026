#include <Arduino.h>
#include "BNO.h"
#include "constants.h"
#include "robot.h"
#include "PID.h"
#include "pixyLib.h"

Pixy2 pixy;
Robot robot;
Bno bno;

#define KP 160/Constants::Motor::maxPWM
#define KI 36/Constants::Motor::maxPWM
#define KD 50/Constants::Motor::maxPWM

float kBallFollowOffsetBack = 1.2;
float kBallFollowOffsetSide = 1.0;
float kBallFollowOffsetFront = 1.0;

#define ERROR_THRESHOLD 100

PID pid(KP, KI, KD, ERROR_THRESHOLD);

double targetYaw = 0.0;

static double WrapAngle180(double deg) {
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

void setup() {
    Serial.begin(115200);
    robot.begin();
    bno.begin();
    delay(1000);
    double initialYaw = bno.GetBNOData();
    delay(100);
    int result = pixy.init();
    if (result == 0) {
        Serial.println("[SUCCESS] Pixy2 connected!");
    } else {
        Serial.println("[FAIL] Pixy2 not found. Check wiring or power");
        while (true);
    }
    delay(1000);
    
}

void loop() {
    static unsigned long lastPixyTime = 0;
    static double ballAngle = 0.0;
    if (millis() - lastPixyTime >= 100) {
        lastPixyTime = millis();
        pixy.ccc.getBlocks();
        bool found = false;
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == 1) {
                found = true;
                uint16_t xComponent = pixy.ccc.blocks[i].m_x;
                uint16_t yComponent = pixy.ccc.blocks[i].m_y;
                double angleDeg = (atan2((double)yComponent, (double)xComponent) * 180.0 / PI) - 45.0;
                ballAngle = angleDeg;
                Serial.print("BALL| x= ");
                Serial.print(xComponent);
                Serial.print("  y= ");
                Serial.print(yComponent);
                Serial.print("  angle= ");
                Serial.println(angleDeg);
            }
        }
        if (!found) {
            Serial.println("No subject detected");
        }
    }

    double yaw = WrapAngle180(bno.GetBNOData());
    double speed = pid.Calculate(targetYaw, yaw);
    double ballAngleRelativeToRobot = ballAngle + yaw;
    Serial.print(" BOut: ");
    Serial.print(ballAngle);
    Serial.print(" DAB: ");
    Serial.print(ballAngleRelativeToRobot);
    Serial.print(" Yaw: ");
    Serial.println(yaw);

    // NOTE: Motor::setSpeed floors to an integer PWM value.
    // If we pass 0.45f directly, PWM becomes 0 and the robot won't translate.
    const float drivePwm = 0.45f * Constants::Motor::maxPWM;
    robot.motors.move(ballAngle, drivePwm, speed);
}