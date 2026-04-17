#include <Arduino.h>
#include "Pixy2I2C.h"

Pixy2I2C pixy;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Pixy2 - Detection Test");
  Serial.println("====================================");
  Serial.println("Starting I2C bus for Pixy...");

  const int8_t status = pixy.init();

  if (status == 0) {
    Serial.println("[SUCCESS] Pixy2 connected!");
  } else {
    Serial.print("ERROR: Pixy not found. Error code: ");
    Serial.println(status);
    Serial.println("Check: 1. PCB traces, 2. swapped SCL/SDA wires, 3. PixyMon I2C mode");
    while (true);
  }
}

void loop() {
  const int8_t blockStatus = pixy.ccc.getBlocks();

  if (blockStatus < 0) {
    Serial.print("Pixy read error: ");
    Serial.println(blockStatus);
  } else if (pixy.ccc.numBlocks) {
    Serial.print("Detected ");
    Serial.print(pixy.ccc.numBlocks);
    Serial.println(" object(s).");

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      const Block& block = pixy.ccc.blocks[i];
      const uint32_t area = static_cast<uint32_t>(block.m_width) * block.m_height;

      Serial.print("Block ");
      Serial.print(i);
      Serial.print(" | sig: ");
      Serial.print(block.m_signature);
      Serial.print(" | pos: (");   
      Serial.print(block.m_x);
      Serial.print(", ");
      Serial.print(block.m_y);
      Serial.print(") | size: ");
      Serial.print(block.m_width);
      Serial.print("x");
      Serial.print(block.m_height);
      Serial.print(" | area: ");
      Serial.println(area);
    }
  } else {
    Serial.println("No subject detected");
  }
  
  delay(100);
}
