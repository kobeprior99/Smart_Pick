#include "Arduino.h"
#include "i2c_helper.h"

fclass::fclass(){}

//helper functions
void fclass::Setup(){
  // Enable capacitance channel (mode)
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  //enable continuous conversion and faster sampling rate
  writeRegister(AD7746_REG_CFG, 0b10000001);
  //enable excitation A and B such that B is the inverse of A
  writeRegister(AD7746_REG_EXC_SETUP, 0x03 | 0b00011000);
  //add the offset
  writeRegister(AD7746_REG_CAPDACA, 0xFF);
}

void fclass::writeRegister(uint8_t subaddress, uint8_t value) {
  //start transmission with CDC
  Wire.beginTransmission(AD7746_ADDRESS);
  //set address pointer register to subaddress
  Wire.write(subaddress);
  //write a byte to the address pointed to above
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t fclass::readRegister(uint8_t subaddress) {
  //start transmission with CDC
  Wire.beginTransmission(AD7746_ADDRESS);
  //set address pointer register to subaddress
  Wire.write(subaddress);
  Wire.endTransmission(false);
  //read from the address pointed to from above
  Wire.requestFrom(AD7746_ADDRESS, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

bool fclass::dataReady() {
  //get RDY bit from status register
  uint8_t status = readRegister(AD7746_REG_STATUS);

  // RDYCAP = 0 means data ready
  return (status & AD7746_STATUS_RDYCAP_MSK) == 0b00000000;
}

long fclass::readCapacitanceRaw() {
  //the device address
  Wire.beginTransmission(AD7746_ADDRESS);
  //register address pointer
  Wire.write(AD7746_REG_CAP_DATA_HIGH);
  Wire.endTransmission(false);
  //read 3 bytes and stich them together, address pointer auto incrememnts
  Wire.requestFrom(AD7746_ADDRESS, 3);
  if (Wire.available() < 3) return -1;  //error
  long raw = ((long)Wire.read() << 16) | ((long)Wire.read() << 8) | ((long)Wire.read());
  return raw & 0xFFFFFF;  //24-bit
}


fclass cdc = fclass();
