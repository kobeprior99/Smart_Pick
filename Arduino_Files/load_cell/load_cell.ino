/*
  ==============================================================================  
  Project: load_cell  
  File:    load_cell.ino
  Authors: Kobe Prior and Sophia Mimlitz  
  Date Created:    November 11, 2025  

  Description:  
  This program samples capacitance values using an ESP32 microcontroller  
  interfaced with the CN0552-PMDZ Capacitance-to-Digital Converter (CDC) board.  
  The ESP32 polls the CDC via I²C (depending on configuration) to measure  
  real-time capacitance for sensor evaluation and data logging applications.  
  
  This program reports data over serial to be plotted live using a python script
  A specific offset is applied for the load cell we've created. 

  Hardware Setup:
    SDA CN0552 -> GPIO 21 ESP32
    SCL CN0552 -> GPIO 22 ESP32
    add 4.7kΩ pull-up resistor to SDA/SCL
  CN0552
    +/- 4.096 pF at maximum bulk capacitance of 17 pF
    can be extended to 50 pF with maximum bulk capacitance of 200 pF
    use 16.1 SPS for 50/60 Hz rejection
    we can probably use 90 sps

    Resolution down to 4aF
    Accuracy: 4 fF
    Common mode (not changing) capacitance up to 17 pF
    Full-scale (changing) capactiance range +/- 4 pF
    Update Rate 10Hz to 90 Hz, why we might need accelarometer for harmonics

  //register definitions come from no os driver header file 
  //https://github.com/analogdevicesinc/no-OS/blob/main/drivers/cdc/ad7746/ad7746.h
  ==============================================================================  
*/

//wire library for i2c
#include "cdc.hpp"

void setup() {
  //start serial
  Serial.begin(115200);
  //setup the ADC see i2c_helper.cpp
  cdc.Setup();

  //TODO accell setup 
}

void loop() {
  //check if data ready then read capacitance
  if (cdc.dataReady()) {
    long raw = cdc.readCapacitanceRaw();
    //TODO: Create Linear Model raw to force 

    Serial.print("Min:");
    Serial.print(0);  //min
    Serial.print(",");
    Serial.print("Capacitance:");
    Serial.print(raw);
    Serial.print(",");
    Serial.print("Max:");
    Serial.println(16777215);  //max
  }
}
