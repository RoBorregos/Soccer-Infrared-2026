
#include <Arduino.h>
#include "goodSerializer.h"

GoodSerializer serializerTest;

void setup(){
    serializerTest.begin();
}

void loop(){
    serializerTest.processLoop();
}