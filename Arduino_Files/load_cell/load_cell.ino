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

//include capacitive to digital converter header file
#include "cdc.hpp"
//include accelerometer header file
#include "accel.hpp"

void setup() {
  //start serial
  Serial.begin(115200);

  //setup the cdc
  cdc.Setup();

  //setup accell
  if (!accel.begin()){
    Serial.println("Failed to initialize MPU6050");
    while(1) delay(10);
  } 
  accel.begin();
}

unsigned long lastMicros = 0;
long lastRaw = 0;  // store last capacitance value

void loop() {
    unsigned long currentMicros = micros();
    
    // check if 1.25 ms have passed (800 Hz)
    if (currentMicros - lastMicros >= 1250) {
        lastMicros += 1250;  // advance to next tick

        // read capacitance if ready
        if (cdc.dataReady()) {
            lastRaw = cdc.readCapacitanceRaw();
        }
        // else keep lastRaw

        // read accelerometer
        float z = accel.getAccelZ();

        // print everything
        Serial.print("Min Capacitance:");
        Serial.print(0);
        Serial.print(",");
        Serial.print("Capacitance:");
        Serial.print(lastRaw);
        Serial.print(",");
        Serial.print("Max Capacitance:");
        Serial.print(16777215);
        Serial.print(",");
        Serial.print("AccelZ:");
        Serial.println(z);
    }
}
