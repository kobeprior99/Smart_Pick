/*
  ==============================================================================  
  Project: Capacitance Sampling using ESP32 and CN0552-PMDZ  
  File:    measure_cap.ino
  Authors: Kobe Prior and Sophia Mimlitz  
  Date:    October 28, 2025  

  Description:  
  This program samples capacitance values using an ESP32 microcontroller  
  interfaced with the CN0552-PMDZ Capacitance-to-Digital Converter (CDC) board.  
  The ESP32 polls the CDC via I²C (depending on configuration) to measure  
  real-time capacitance for sensor evaluation and data logging applications.  
  
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


  ==============================================================================  
*/
#include header files
#include <Wire.h>
// Define Variables

//CDC address
#define AD7746_ADDRESS 0x48// Default I2C address for AD7746
//Regiser addresses
#define REG_STATUS 0x07
#define REG_CAP_DATA_H 0x00
#define REG_CAP_DATA_M 0x01
#define REG_CAP_DATA_L 0x02

void setup() {
  Serial.begin(115200);
  Wire.begin(); //SDA=21, SCL=22 by default for ESP32
  Serial.println('CN0052 (AD7746) Capacitance Measurement Starting...');
}

void loop() {

}
