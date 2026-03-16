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
#include "flash/flash.hpp"
#include "accel/Accelerometer.hpp"
#include "cdc/CDCConverter.hpp"
#include "leds/LEDs.hpp"
#include "utils/DataTypes.hpp"
#include <Wire.h>
TwoWire I2C_CDC = TwoWire(0);  //bus for CDC
TwoWire I2C_ACC = TwoWire(1);  //bus for accelerometer
                               //
LEDs leds;                     //create a LED object
CDC cdc(I2C_CDC);              //create a cdc object passing in the respective TwoWire object
ACC acc(I2C_ACC);              //create accelerometer object passivng in the correct two wire object
uint32_t currentCapacitance = 0;

bool recording = false;

flash<32> flashLogger;  // 32 packets per double-buffer
#define LOG_BASE_ADDR 0x000000
#define LOG_SIZE_BYTES 0x1000000  // 16MB — full 128Mbit chip
uint32_t startTime = 0;

void checkSerialCommand() {
  if (Serial.available() > 0) {
    //Serial.println("Type R and press Enter to Read");
    char cmd = Serial.read();
    // 'R' — read all logged packets back over serial
    if (cmd == 'R' || cmd == 'r') {
      leds.Read_Data();
      uint32_t totalBytes = flashLogger.getPacketsLogged() * flashLogger.getPacketSize();
      FlashPacket fp;
      Serial.println("timestamp_us,acceleration,capacitance");
      for (uint32_t addr = LOG_BASE_ADDR; addr < LOG_BASE_ADDR + totalBytes; addr += sizeof(FlashPacket)) {
        flashLogger.read(addr, (uint8_t*)&fp, sizeof(FlashPacket));
        Serial.print(fp.timestamp);
        Serial.print(",");
        Serial.print(fp.mag_accel);
        Serial.print(",");
        Serial.println(fp.capacitance);
      }
      leds.End_Read_Data();
    }

    // 'E' — end recording and finalize
    if (cmd == 'E' || cmd == 'e') {
      leds.Read_Data();
      flashLogger.finalizeLog();
      recording = false;
      Serial.print("Recording ended. Packets logged: ");
      Serial.println(flashLogger.getPacketsLogged());
      leds.End_Read_Data();
    }

    // 'D' — erase flash
    if (cmd == 'D' || cmd == 'd') {
      leds.Read_Data();
      Serial.println("Erasing flash...");
      flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
      flashLogger.eraseLogRegion();
      Serial.println("Flash erased.");
      leds.End_Read_Data();
    }
  }
}


void setup() {
  //basically set all input and output pins and flash them so we know they work
  leds.Init();
  //USB serial instead of normal (make sure tools > USB CDC on boot)
  Serial.begin(115200);
  I2C_CDC.begin(11, 10, 400000);  // SDA, SCL, baud
  I2C_ACC.begin(41, 40, 400000);  // 400kHz baud necesary to get high required samp rate.
  cdc.Setup();
  acc.Setup();
}

void loop() {
  //start every loop by checking if the the user wants to read
  checkSerialCommand();

  //wait until interrupt on accelerometer to know the test has begun
  if (!recording && acc.Interrupt1()) {
    recording = true;
    startTime = micros();
    leds.Start_Recording();
  }

  if (recording) {
    //we're okay to start collecting data here
    //we will do a "zero order hold on capacitance so it doesn't bog down the sample rate"
    if (cdc.dataReady()) {
      currentCapacitance = cdc.readCapacitanceRaw();
    }
    FlashPacket fp;
    fp.timestamp = micros() - startTime;
    fp.mag_accel = acc.readAccelMagnitude();// adjust field name to match your ACC type
    fp.capacitance = currentCapacitance;

    if (!flashLogger.appendPacket(fp)) {
      Serial.println("WARNING: Flash overflowed or write failed");
      recording = false;
      leds.End_Recording();  // add whatever your LED stop call is
    }

    // Service the write EVERY loop — this is what actually pushes data to flash
    flashLogger.serviceFlashWrite();
  }
  
}
