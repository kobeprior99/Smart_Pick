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
// create cdc object
CDC cdc;
//include accelerometer header file
#include "accel.hpp"
// create accelerometer object
Accel accel;

#include "buffer.hpp"
//create double bufferReady
constexpr size_t BUFFER_LEN = 512;
DoubleBuffer<BUFFER_LEN> dbuf;
//store the lastRaw capacitance value
long lastRaw = 0;

void setup() {
  //start serial
  Serial.begin(115200);
  Wire.begin();// <-- explicitly start I2C ONCE

  //setup the cdc
  cdc.Setup();

  //setup accell
  if (!accel.begin()){
    Serial.println("Failed to initialize MPU6050");
    while(1) delay(10);
  } 
}

void loop() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= 1250) {
    lastMicros += 1250;

    long cap = lastRaw;
    if (cdc.dataReady()) {
      cap = cdc.readCapacitanceRaw();
      lastRaw = cap;
    }

    float az = accel.getAccelZ();
    dbuf.push(az, cap); // assumes bounds-checked
  }

  if (dbuf.available()) {
    const auto* data = dbuf.read();
    for (size_t i = 0; i < dbuf.size(); i++) {
      Serial.print(data[i].t_us);
      Serial.print(',');
      Serial.print(data[i].capRaw);
      Serial.print(',');
      Serial.println(data[i].accelZ, 6);
    }
  }
}
