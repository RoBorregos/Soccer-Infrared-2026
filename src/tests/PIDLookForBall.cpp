// #include <Arduino.h>
// #include "BNO.h"
// #include "constants.h"
// #include "robot.h"
// #include "PID.h"
// #include "IRRing.h"

// IRRing irring;
// Robot robot;
// Bno bno;

// #define KP 180/Constants::Motor::maxPWM
// #define KI 42/Constants::Motor::maxPWM
// #define KD 50/Constants::Motor::maxPWM

// float kBallFollowOffsetBack = 1.2;
// float kBallFollowOffsetSide = 1.0;
// float kBallFollowOffsetFront = 1.0;

// #define ERROR_THRESHOLD 100

// PID pid(KP, KI, KD, ERROR_THRESHOLD);

// double targetYaw = 0.0;
// double BNOCORRECTION = 0.0;

// static double WrapAngle180(double deg) {
//     deg = fmod(deg + 180.0, 360.0);
//     if (deg < 0) deg += 360.0;
//     return deg - 180.0;
// }

// void setup() {
//     Serial.begin(115200);
//     robot.begin();
//     bno.begin();
//     delay(1000);
//     double initialYaw = bno.GetBNOData();
//     unsigned long currentTime = millis();
//     irring.init(&currentTime);
//     irring.SetOffset(0.0);
//     irring.UpdateData();
//     delay(1000);
    
// }

// void loop() {
//     irring.UpdateData();
//     double yaw = WrapAngle180(bno.GetBNOData() + BNOCORRECTION);
//     // TEMP SIGN CORRECTION
//     double ballAngle = -irring.GetAngle(kBallFollowOffsetBack, kBallFollowOffsetSide, kBallFollowOffsetFront);
//     double speed = pid.Calculate(targetYaw, yaw);
//     double ballAngleRelativeToRobot = ballAngle + yaw;
//     Serial.print(" BOut: ");
//     Serial.print(ballAngle);
//     Serial.print(" DAB: ");
//     Serial.print(ballAngleRelativeToRobot);
//     Serial.print(" Yaw: ");
//     Serial.println(yaw);

//     // NOTE: Motor::setSpeed floors to an integer PWM value.
//     // If we pass 0.35f directly, PWM becomes 0 and the robot won't translate.
//     const float drivePwm = 0.35f * Constants::Motor::maxPWM;
//     robot.motors.move(ballAngle, drivePwm, speed);
// }