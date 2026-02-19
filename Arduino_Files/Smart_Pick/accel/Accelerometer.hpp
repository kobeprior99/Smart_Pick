#pragma once
#include <Arduino.h>
#include <Wire.h>

struct AccelData {
    float x;
    float y;
    float z;
};

class Accelerometer {
public:
    void begin();
    void poll();
    AccelData getData();

private:
    AccelData currentData;
    void readRaw();
};
