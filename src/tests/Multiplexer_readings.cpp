#include <Arduino.h>
#include "Multiplexer.h" 
#include "constants.h"   

Multiplexer left_mux_(Constants::kSignalPin1, Constants::kMUXPin1_1, Constants::kMUXPin2_1, Constants::kMUXPin3_1);
Multiplexer right_mux_(Constants::kSignalPin2, Constants::kMUXPin1_2, Constants::kMUXPin2_2, Constants::kMUXPin3_2);
Multiplexer front_mux_(Constants::kSignalPin3, Constants::kMUXPin1_3, Constants::kMUXPin2_3, Constants::kMUXPin3_3);

void setup() {
    Serial.begin(115200); 
    
    left_mux_.InitializeMultiplexer();
    right_mux_.InitializeMultiplexer();
    front_mux_.InitializeMultiplexer();
    
    // Wait
    delay(1000);
    Serial.println("Starting MUX test enviroment...");
}

void loop() {
    Serial.println("Ch\tLeft\tRight\tFront");

    for (uint8_t i = 0; i < 8; i++) {

        int leftValue  = left_mux_.readChannel(i);
        int rightValue = right_mux_.readChannel(i);
        
        // Front only reads if active channel is 0, 1, 2, 3, 4, or 5
        int frontValue = 0; 
        bool frontActive = (i < 6); 

        if (frontActive) {
            frontValue = front_mux_.readChannel(i);
        }

        Serial.print(i);
        Serial.print("\t");
        Serial.print(leftValue);
        Serial.print("\t");
        Serial.print(rightValue);
        Serial.print("\t");

        if (frontActive) {
            Serial.println(frontValue);
        } else {
            Serial.println("---"); 
        }
    }

    Serial.println(); 
    delay(350);
}

