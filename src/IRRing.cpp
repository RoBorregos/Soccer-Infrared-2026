#include "sensor_control.h"
#include <Arduino.h>

/**
 * # Ciclo de la pelota
 * $$ \displaystyle \frac{\text{Número de sensores}}{10,000\ \mathrm{ms}} = \text{tiempo de ciclo} $$
 * */ 
#define T_MODEA 714
unsigned long time_ms = 0;

void printComponents(vectorXY_t *vectorXY_p) {
    Serial.print("X component: ");
    Serial.print(vectorXY_p->x);
    Serial.print("Y component: ");
    Serial.print(vectorXY_p->y);
  }

void printAngulo(vectorRT_t *self) {
  Serial.print("a ");
  Serial.print(self->theta);
  Serial.print("\n");
}

void setup() {
    Serial.begin(115200);
    setAllSensorPinsInput();
}


void loop() {

  //Inicio de variables
    float           pulseWidth[IR_NUM]; 
    sensorInfo_t    sensorInfo;         
    vectorXY_t      vectorXY;          
    vectorRT_t      vectorRT;          
    vectorRT_t      vectorRTWithSma;    
    
    sensorInfo  = getAllSensorPulseWidth(pulseWidth, T_MODEA);
    vectorXY    = calcVectorXYFromPulseWidth(pulseWidth);
    vectorRT    = calcRTfromXY(&vectorXY);


//Imprimir el radio (distancia y el ángulo
    if (millis() - time_ms > 50) {
        time_ms = millis();
      
      // printComponents(&vectorRT);
      printAngulo(&vectorRT);
      Serial.print("r ");
      Serial.print(sensorInfo.avgPulseWidth);
      Serial.print("\n");

    }
}