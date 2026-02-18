#include "robot.h"

auto left = Motor(1,
    Constants::Motor::Left::pwm,
    Constants::Motor::Left::in1,
    Constants::Motor::Left::in2
);
        
auto center = Motor(2,
    Constants::Motor::Center::pwm,
    Constants::Motor::Center::in1,
    Constants::Motor::Center::in2
);

auto right = Motor(3,
    Constants::Motor::Right::pwm,
    Constants::Motor::Right::in1,
    Constants::Motor::Right::in2
);

void moveMotor(Motor &motor, int speed) {
    motor.setSpeed(speed);
    delay(1000);
    motor.stop();
}

void setup(){
    
}

void loop() {
    delay(2000);
    moveMotor(left, 100);
    delay(1000);
    moveMotor(center, 100);
    delay(1000);
    moveMotor(right, 100);
}