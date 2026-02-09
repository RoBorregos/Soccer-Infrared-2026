#include <Wire.h>
#include <Arduino.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Escaneando I2C...");
}

void loop() {
  byte error, address;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      Serial.println(address, HEX);
    }
  }
  delay(5000);
}
