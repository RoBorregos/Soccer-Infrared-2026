// communicationWithTeensy.cpp
// THIS WILL RUN ON THE TEENSY 4.1
#include <Arduino.h>

// Teensy 4.1 Hardware Serial 3 (Pin 15 = RX3, Pin 14 = TX3)
#define HWSERIAL Serial3 

void setup() {
  // USB Serial to PC
  Serial.begin(115200); 
  
  // Set Teensy to 58824 to flawlessly match the Uno's actual hardware baud rate
  HWSERIAL.begin(9600); 
}

void loop() {
  // Read string from Arduino Uno via UART and print to PC
  if (HWSERIAL.available() > 0) {
    String incomingData = HWSERIAL.readStringUntil('\n'); 
    incomingData.trim(); // Remove lingering \r carriage returns
    
    if (incomingData.length() > 0) {
      Serial.print("UART received Theta: ");
      Serial.println(incomingData);
    }
  }

  // Echo anything typed in the PC Serial Monitor back to the Uno (if needed)
  if (Serial.available() > 0) {
    String usbData = Serial.readStringUntil('\n');
    usbData.trim();
    
    HWSERIAL.println(usbData);
    Serial.print("USB sent to Uno: ");
    Serial.println(usbData);
  }
}