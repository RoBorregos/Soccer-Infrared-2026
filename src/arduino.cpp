#include <Arduino.h>
#include <ArduinoSTL.h>
// #include <vector>
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

Motors motors(
    kMotor1Pwm, kMotor1In1, kMotor1In2,
    kMotor2Pwm, kMotor2In1, kMotor2In2,
    kMotor3Pwm, kMotor3In1, kMotor3In2
);

void setup() {
    Serial.begin(9600);
    motors.InitializeMotor();
    bno.InitializeBNO();
}

void loop() {
    float yaw = bno.GetBNOData();
    Serial.print("Yaw: ");
    Serial.println(yaw);

    // motors.MoveRobot(0.15, FORWARD, UL);
    // delay(1000);
    // motors.StopMotor();
    // delay(1000);

    // motors.MoveRobot(0.15, BACKWARD, UL); 
    // delay(1000);
    // motors.StopMotor();
    // delay(1000);
    
}