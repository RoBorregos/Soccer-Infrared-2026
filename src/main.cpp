#include <Arduino.h>
#include "BNO.h"
#include "motor.h"
#include "motors.h"

uint8_t switchPin = 42;

Motors motors(
    kMotor1Pwm, kMotor1In1, kMotor1In2,
    kMotor2Pwm, kMotor2In1, kMotor2In2,
    kMotor3Pwm, kMotor3In1, kMotor3In2);

Bno bno;


void setup() {
  Serial.begin(9600);
  motors.InitializeMotors(switchPin);
  bno.InitializeBNO();
}

void loop() {
  // motors.StartStopMotors(switchPin); // Switch to digital pin
  // motors.SetAllSpeeds(10);
  float yaw = bno.GetBNOData();
  Serial.print("Yaw: ");
  Serial.println(yaw);
//   motors.MoveUL(0, 3, 1);
//   motors.MoveUR(0, 3, 1);
//   motors.MoveLC(0, 3, 1);
//   Serial.println("Moving motors...");
//   delay(500);
//   motors.StopAllMotors();
// delay(500);
}
