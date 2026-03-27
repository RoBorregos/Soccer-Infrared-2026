#include "IRRing.h"  
#include <Arduino.h>
#include <SingleEMAFilterLib.h>

SingleEMAFilter<double> filterAngle(0.6);
SingleEMAFilter<double> filterStr(0.6);
// #define BALLANGLECORRECTION 235.0-45.0
#define BALLANGLECORRECTION 0.0

IRRing::IRRing():
angle(0),
strength(0),
offset(0),
current_time(nullptr),
last_time(0)
{

}
void IRRing::init(unsigned long*current_time)
{
    this->current_time=current_time;
    Serial1.begin(115200);
    Serial1.setTimeout(100);
}



/**
 * [TODO] - Do binary serialization data for extra security and to avoid parsing issues. 
 *  
 * For example, we could send a byte for the type (angle or strength) followed by a 4-byte float for the value. 
 * This would be more compact and less error-prone than sending strings. We would need to update both the sender 
 * and receiver code to handle this new format.
 * 
 * */ 
void IRRing::UpdateData() {
    if (Serial1.available()) {
        String data = Serial1.readStringUntil('\n');
        data.trim();  
        
        // Data is expected to be received in the format: 
        // “a 0.1” for angle or “r 10” for force.
        
        if (data.length() > 2) {  
            char type = data[0];
            String valueStr = data.substring(2);  
            if (valueStr.length() > 0) {  
                double value = valueStr.toFloat();  
                if (type == 'a') {
                    angle = value + offset;
                    filterAngle.AddValue(angle);
                } else if (type == 'r') {
                    strength = value;
                    filterStr.AddValue(strength);
                }
            }
        }
    }
    last_time = *current_time;
}

void IRRing::SetOffset(double offset){
    this->offset=offset;
}
double IRRing::GetRawAngle(){
    if(angle > 180){
        angle -= 360;
    }
    return angle;
}
double IRRing::GetStrength(){
    return filterStr.GetLowPass();
}

// We adjust the angle using different offsets based on how far the ball is from the front.
// Larger angles (ball behind) get a smaller correction using ballFollowOffsetBack,
// medium angles (ball to the side) use ballFollowOffsetSide,
// and smaller angles (ball in front) use ballFollowOffsetFront.
// This allows more precise control depending on the ball’s position.

double IRRing::GetAngle(float ballFollowOffsetBack, float ballFollowOffsetSide, float ballFollowOffsetFront) {
    double currentAngle = angle;
    currentAngle = fmod(currentAngle + 180.0, 360.0);
    if (currentAngle < 0) currentAngle += 360.0;
    currentAngle -= 180.0;

    float magnitude = abs(currentAngle);
    if (magnitude > 52) {
        currentAngle *= ballFollowOffsetBack;
    } else if (magnitude > 25) {
        currentAngle *= ballFollowOffsetSide;
    } else {
        currentAngle *= ballFollowOffsetFront;
    }

    // Noise reduction
    if (magnitude > 0.05) {
        lastBallAngle = currentAngle;
    } else {
        currentAngle = lastBallAngle;
    }

    double finalResult = (currentAngle * -1) + BALLANGLECORRECTION;

    // WRAPPER
    finalResult = fmod(finalResult + 180.0, 360.0);
    if (finalResult < 0) finalResult += 360.0;
    
    return finalResult - 180.0;
}
//Para implementar el offset usando el PID, no sería necesario tener condicionales porque el PID se encarga de corregir el offset según el ángulo
// double IRRing::GetAngle(){
//     if (angle > 180){
//         angle -= 360;
//     }
//     return angle*-1;
// }
// double IRRing::GetAnglewithOffset(float offset){
//     angle = angle * ballFollowOffset;
    
//     return angle;
// }
//Se me ocurre tener dos funciones diferentes, porque para calcular el PID necesitamos el ángulo que ya está corregido para que este en valores de -180 a 180