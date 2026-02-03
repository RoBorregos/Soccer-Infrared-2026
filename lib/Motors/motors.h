#ifndef MOTORS_H
#define MOTORS_H
#pragma once

#define MOTORS_AMOUT 3

// Tentative; untested.
#define LC 0
#define UL 1
#define UR 2

class Motors {
public:
    Motors(int kMotor1Pwm, int kMotor1In1, int kMotor1In2,
        int kMotor2Pwm, int kMotor2In1, int kMotor2In2,
        int kMotor3Pwm, int kMotor3In1, int kMotor3In2);
    void InitializeMotor();
    void RotateRobot(float speedPercent, bool direction);
    void MoveRobot(float speedPercent, bool direction, int exceptIndex);
    void StopMotor();
    void moveRobotOmnidirectional(float angleDegrees, float speedPercent, float rotationalSpeed);

private:
    int _kMotor1Pwm; int _kMotor1In1; int _kMotor1In2;
    int _kMotor2Pwm; int _kMotor2In1; int _kMotor2In2;
    int _kMotor3Pwm; int _kMotor3In1; int _kMotor3In2;

    int motor1; int motor2; int motor3;
};  

#endif // MOTORS_H