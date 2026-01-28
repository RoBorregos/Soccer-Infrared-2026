#ifndef MOTORS_H
#define MOTORS_H
#pragma once

class Motors {
public:
    Motors(int kMotor1Pwm, int kMotor1In1, int kMotor1In2,
        int kMotor2Pwm, int kMotor2In1, int kMotor2In2,
        int kMotor3Pwm, int kMotor3In1, int kMotor3In2);
    void InitializeMotor();
    void MoveMotor(float speedPercent, bool direction);
    void StopMotor();

private:
    int _kMotor1Pwm; int _kMotor1In1; int _kMotor1In2;
    int _kMotor2Pwm; int _kMotor2In1; int _kMotor2In2;
    int _kMotor3Pwm; int _kMotor3In1; int _kMotor3In2;

    int motor1; int motor2; int motor3;
};  

#endif // MOTORS_H