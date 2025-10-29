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


  //register definitions come from no os driver header file 
  //https://github.com/analogdevicesinc/no-OS/blob/main/drivers/cdc/ad7746/ad7746.h
  ==============================================================================  
*/

//wire library for i23
#include <Wire.h>

#define AD7746_ADDRESS 0x48  // 7-bit I2C address

#define REG_STATUS_CAP 0x00
#define REG_CAP_SETUP  0x07
#define REG_EXC_SETUP  0x08
#define REG_CONFIG     0x09

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Give device a moment after power-up
  delay(100);

  // Configure excitation (enable EXCA+)
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(REG_EXC_SETUP);
  Wire.write(0b00010001); // Enable EXCA, default freq
  Wire.endTransmission();

  // Configure capacitance measurement: enable CH1, single-ended
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(REG_CAP_SETUP);
  Wire.write(0b00010001); // Enable CAP channel A, single-ended, CAPEN=1
  Wire.endTransmission();

  // Configure continuous conversion, default data rate
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(REG_CONFIG);
  Wire.write(0b00000000); // Continuous mode, default gain/rate
  Wire.endTransmission();

  Serial.println("CN0552 (AD7746) configured and ready.");
}

bool dataReady() {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(REG_STATUS_CAP);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 1);
  if (Wire.available()) {
    byte status = Wire.read();
    return (status & 0x80); // D7=1 means new CAP data ready
  }
  return false;
}

long readCapacitanceRaw() {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(REG_STATUS_CAP);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 3);
  long capData = 0;
  if (Wire.available() >= 3) {
    capData = ((long)Wire.read() << 16) | ((long)Wire.read() << 8) | Wire.read();
  }
  return capData;
}

void loop() {
  if (dataReady()) {
    long raw = readCapacitanceRaw();
    double capacitance = (raw / 268435456.0 - 1.0) * 8.192e-12; // convert to farads (from datasheet)
    Serial.print("Capacitance: ");
    Serial.print(capacitance * 1e12, 3);
    Serial.println(" pF");
  }
}
