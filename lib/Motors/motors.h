#ifndef MOTORS_H
#define MOTORS_H
#include <vector>
#include <Arduino.h>
#include <ArduinoSTL.h>
#include <memory>

#include "constants.h"

#define MOTORS_AMOUT 3

// Tentative; untested.
#define LC 0
#define UL 1
#define UR 2

class Motors {
public:
    Motors();

    struct Motor {
        int pwmPin;
        int in1Pin;
        int in2Pin;

        void begin();
        void setSpeed(float speed);
        void stop();
    };

    void begin();
    void stop();
    void move(float angleDegrees, float speed, float rotationalSpeed = 0);

    Motor motor1;
    Motor motor2;
    Motor motor3;

private:
};  

#endif // MOTORS_H