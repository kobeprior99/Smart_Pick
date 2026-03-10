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
LEDs leds;  //initialize an instance of the LED clasUSBSerial SerialUSB;
void checkSerialCommand() {
  if (Serial.available() > 0) {
    Serial.println("Type R and press Enter to Read");
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      leds.Read_Data();
      Serial.print("TODO: we see you requested to read");
      delay(2000);
      leds.End_Read_Data();
    }
  }
}


void setup() {
  //USB serial instead of normal (make sure tools > USB CDC on boot)
  Serial.begin(115200);
  leds.Init();  //basically set all input and output pins and flash them so we know they work
}

void loop() {
  //start every loop by checking if the the user wants to read
  // checkSerialCommand();

  //wait until interrupt on accelerometer to know the test has begun
  //TODO accelerometer interrupt
  //TODO turn on RED LED when we start recordingk
}
