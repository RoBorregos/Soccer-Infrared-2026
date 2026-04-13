#include <Arduino.h>
#include <Pixy2.h>

Pixy2 pixy;

#define SIG_ORANGE_BALL  1
#define SIG_YELLOW_GOAL 2

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Pixy2 - Ball Detection Test");
  Serial.println("====================================");

  int result = pixy.init();

  if (result == 0) {
    Serial.println("[SUCCESS] Pixy2 connected!");
  } else {
    Serial.println("[FAIL] Pixy2 not found. Check wiring, pixyMon interface configs, energy supply or whatever idk lol");
    while (true); 
  }
}

void loop() {
  pixy.ccc.getBlocks();

  bool found = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIG_YELLOW_GOAL) {
      found = true;
      Serial.print("GOAL| x= ");
      Serial.print(pixy.ccc.blocks[i].m_x);
      Serial.print("  y= ");
      Serial.print(pixy.ccc.blocks[i].m_y);
      Serial.print("  size= ");
      Serial.print(pixy.ccc.blocks[i].m_width);
      Serial.print("x");
      Serial.println(pixy.ccc.blocks[i].m_height);
    }
  }

  if (!found) {
    Serial.println("No subject detected");
  }

  delay(200);
}