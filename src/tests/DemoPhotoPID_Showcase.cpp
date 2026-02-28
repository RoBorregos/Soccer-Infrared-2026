#include "photo.h"
#include "robot.h"
#include "PID.h"
#include "BNO.h"

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting Showcase...");
    delay(1000);

    Robot robot;
    Bno bno;
    robot.begin();
    bno.begin();

    
}

void loop() {

    // double yaw = bno.GetBNOData();
    // Serial.print("Yaw: ");
    // Serial.print(yaw);
    // Serial.print(" | Target Yaw: ");
    // double targetYaw = 0.0;
    // double pidOutput = pid.Calculate(targetYaw, yaw);
    // Serial.print(" Target Yaw:");
    // Serial.print(targetYaw);
    // Serial.print(" Yaw: ");
    // Serial.println(yaw);
    // robot.move(0, 0.35f, pidOutput);

    // robot.move(0, 50);
    // delay(500); 

    // robot.move(90, 50);
    // delay(250);

}


