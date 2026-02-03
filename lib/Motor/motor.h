#ifndef MOTOR_H
#define MOTOR_H

#define FORWARD 1
#define BACKWARD 0

class Motor {
public:
    Motor(int pwmPin, int in1Pin, int in2Pin);
    void InitializeMotor();
    void MoveMotor(float speedPercent, bool direction);
    void StopMotor();
    void SetPWM(const uint8_t pwm);
    void SetSpeed(float speedPercent);
private:
    int pwmPin_;
    int in1Pin_;
    int in2Pin_;
};

#endif // MOTOR_H