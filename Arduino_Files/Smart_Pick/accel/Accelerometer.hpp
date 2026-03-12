#ifndef CDC_HPP
#define CDC_HPP 

#include <Arduino.h>
#include <Wire.h>

struct AccelData {
    float x;
    float y;
    float z;
};

class ACC {
public:
  (TwoWire &wire) : _wire(wire){}
  void Setup(){
    //setup code here
  }
  bool Interrupt(){
    //wait for interrupt pin to go high
  }
private:
    AccelData currentData;
    void readRaw();
};

#endif
