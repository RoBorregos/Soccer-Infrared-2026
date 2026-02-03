#include <Arduino.h>
#include <ArduinoSTL.h>
// #includex <vector>
#include "BNO.h"
#include "constants.h"
#include "motor.h"
#include "motors.h"

// Motors motors(
//     kMotor1Pwm, kMotor1In1, kMotor1In2,
//     kMotor2Pwm, kMotor2In1, kMotor2In2,
//     kMotor3Pwm, kMotor3In1, kMotor3In2
// );
Bno bno;

Motor motor1(kMotor1Pwm, kMotor1In1, kMotor1In2);
Motor motor2(kMotor2Pwm, kMotor2In1, kMotor2In2);
Motor motor3(kMotor3Pwm, kMotor3In1, kMotor3In2);

// Motors motors(
//     kMotor1Pwm, kMotor1In1, kMotor1In2,
//     kMotor2Pwm, kMotor2In1, kMotor2In2,
//     kMotor3Pwm, kMotor3In1, kMotor3In2
// );

void setup() {
    Serial.begin(9600);
    motor1.InitializeMotor();
    motor2.InitializeMotor();
    motor3.InitializeMotor();
    bno.InitializeBNO();
    Serial.println("Setup complete");
    delay(5000); // Prevenir que el robot se suicide

}

void loop() {
    // float yaw = bno.GetBNOData();
    // Serial.print("Yaw: ");
    // Serial.println(yaw);
    motor1.MoveMotor(30.0, FORWARD);
    delay(1000);
    motor1.StopMotor();
    motor2.MoveMotor(30.0, FORWARD);
    delay(1000);
    motor2.StopMotor();
    motor3.MoveMotor(30.0, FORWARD);
    delay(1000);
    motor3.StopMotor(); // This one moves counterclockwise
    
}