#pragma once
#include <Arduino.h>
#include <Wire.h>

class CDCConverter {
public:
    void begin();
    uint32_t readCapacitance();
    float getCapacitancePF();

private:
    uint32_t rawValue;
};
