/*
   ================================================================
   Title:        Smart Pick For Rock Mining 
   Date:         February 15, 2026
   Authors:      Kobe Prior, Sophia Mimlits

   Brief:
   This code collects load cell data via capacitance change and 
   acceleration data from an onboard accelerometer. The collected 
   data is formatted and logged to flash memory for later retrieval 
   and interpretation using Read_Data.ino.
   ================================================================
*/
#include "config/PinConfig.hpp"
#include "flash/FlashMemory.hpp"
#include "accel/Accelerometer.hpp"
#include "cdc/CDCConverter.hpp"
#include "leds/LEDs.hpp"
#include "utils/DataTypes.hpp"
#include <Wire.h>
TwoWire I2C_CDC = TwoWire(0); //bus for CDC
TwoWire I2C_ACC = TwoWire(1); //bus for accelerometer
                              //
LEDs leds;  //create a LED object 
CDC cdc(I2C_CDC); //create a cdc object passing in the respective TwoWire object 
ACC acc(I2C_ACC)
uint32_t currentCapacitance= 0;
bool recording = false;
void checkSerialCommand() {
  if (Serial.available() > 0) {
    //Serial.println("Type R and press Enter to Read");
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      leds.Read_Data();
      Serial.print("TODO: we see you requested to read");
      //replace serial print above with the task of reading things to from memory
      delay(2000);
      leds.End_Read_Data();
    }

    if (cmd == 'E' || cmd == 'e') {
      leds.Read_Data(); //flash the same led twice
      Serial.print("TODO: add code to erase the flash data");
      leds.End_Read_Data();
      delay(2000);
    }
  }
}


void setup() {
  //basically set all input and output pins and flash them so we know they work
  leds.Init();
  //USB serial instead of normal (make sure tools > USB CDC on boot)
  Serial.begin(115200);
  I2C_CDC.begin(11, 10, 400000);  // SDA, SCL, baud
  I2C_ACC.begin(41, 40, 400000);    // 400kHz baud necesary to get high required samp rate.  
  cdc.Setup();
  acc.Setup();
}

void loop() {
  //start every loop by checking if the the user wants to read
  checkSerialCommand();
   
  //wait until interrupt on accelerometer to know the test has begun
  if(!recording && acc.Interrupt1()){
    recording = true;
    leds.Start_Recording();
  }

  if(recording){
    //we're okay to start collecting data here
    //we will do a "zero order hold on capacitance so it doesn't bog down the sample rate"
    if(cdc.dataReady()){
      currentCapacitance = cdc.readCapacitanceRaw();
    }
    SensorPacket pkt;
    pkt.timestamp = micros() - startTime;
    pkt.capacitance = currentCapacitance;
    pkt.acceleration = acc.read();
    
    //TODO log the packet 
  }
}
