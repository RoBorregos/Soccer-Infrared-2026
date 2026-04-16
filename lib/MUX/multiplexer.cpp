#include "Multiplexer.h"

Multiplexer::Multiplexer(uint8_t signalPin, uint8_t s0, uint8_t s1, uint8_t s2)
    : signal_pin_(signalPin), s0_(s0), s1_(s1), s2_(s2)  {}

void Multiplexer::InitializeMultiplexer() {
    pinMode(s0_, OUTPUT);
    pinMode(s1_, OUTPUT);
    pinMode(s2_, OUTPUT);
    pinMode(signal_pin_, INPUT);

    digitalWrite(s0_, LOW);
    digitalWrite(s1_, LOW);
    digitalWrite(s2_, LOW);
} 

void Multiplexer::selectChannel(uint8_t channel) {
    channel &= 0x07;
    digitalWrite(s0_, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(s1_, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(s2_, (channel & 0x04) ? HIGH : LOW);
}

float Multiplexer::readChannel(uint8_t channel) {
    selectChannel(channel);
    
    delayMicroseconds(100);
    analogRead(signal_pin_);
    delayMicroseconds(50);

    return analogRead(signal_pin_);
} 
