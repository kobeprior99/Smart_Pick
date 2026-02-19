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
void setup(){
  flash.begin();
  accel.begin();
  cdc.begin();
  leds.begin();
}

void loop(){
  //wait until interrupt on accelerometer to know the test has begun
  //
}
