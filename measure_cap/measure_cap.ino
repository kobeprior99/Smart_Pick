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
//functions to generate bit masks
#define NO_OS_BIT(x) (1U << (x))
#define NO_OS_GENMASK(h, l) (((0xFF << (l)) & (0xFF >> (7 - (h)))))
//define registers from the driver file:
/* AD7746 Slave Address */
#define AD7746_ADDRESS			0x48

/* AD7746 Reset command */
#define AD7746_RESET_CMD		0xBF

/* AD7746 Register Definition */
#define AD7746_REG_STATUS		0u
#define AD7746_REG_CAP_DATA_HIGH	1u
#define AD7746_REG_CAP_DATA_MID		2u
#define AD7746_REG_CAP_DATA_LOW		3u
#define AD7746_REG_VT_DATA_HIGH		4u
#define AD7746_REG_VT_DATA_MID		5u
#define AD7746_REG_VT_DATA_LOW		6u
#define AD7746_REG_CAP_SETUP		7u
#define AD7746_REG_VT_SETUP		8u
#define AD7746_REG_EXC_SETUP		9u
#define AD7746_REG_CFG			10u
#define AD7746_REG_CAPDACA		11u
#define AD7746_REG_CAPDACB		12u
#define AD7746_REG_CAP_OFFH		13u
#define AD7746_REG_CAP_OFFL		14u
#define AD7746_REG_CAP_GAINH		15u
#define AD7746_REG_CAP_GAINL		16u
#define AD7746_REG_VOLT_GAINH		17u
#define AD7746_REG_VOLT_GAINL		18u

#define AD7746_NUM_REGISTERS		(AD7746_REG_VOLT_GAINL + 1u)

/* AD7746_REG_STATUS bits */
#define AD7746_STATUS_EXCERR_MSK	NO_OS_BIT(3)
#define AD7746_STATUS_RDY_MSK		NO_OS_BIT(2)
#define AD7746_STATUS_RDYVT_MSK		NO_OS_BIT(1)
#define AD7746_STATUS_RDYCAP_MSK	NO_OS_BIT(0)

/* AD7746_REG_CAP_SETUP bits */
#define AD7746_CAPSETUP_CAPEN_MSK	NO_OS_BIT(7)
#define AD7746_CAPSETUP_CIN2_MSK	NO_OS_BIT(6)
#define AD7746_CAPSETUP_CAPDIFF_MSK	NO_OS_BIT(5)
#define AD7746_CAPSETUP_CAPCHOP_MSK	NO_OS_BIT(0)

/* AD7746_REG_VT_SETUP bits */
#define AD7746_VTSETUP_VTEN_MSK		NO_OS_BIT(7)
#define AD7746_VTSETUP_VTMD_MSK		NO_OS_GENMASK(6,5)
#define AD7746_VTSETUP_EXTREF_MSK	NO_OS_BIT(4)
#define AD7746_VTSETUP_VTSHORT_MSK	NO_OS_BIT(1)
#define AD7746_VTSETUP_VTCHOP_MSK	NO_OS_BIT(0)

/* AD7746_REG_EXC_SETUP bits */
#define AD7746_EXCSETUP_CLKCTRL_MSK	NO_OS_BIT(7)
#define AD7746_EXCSETUP_EXCON_MSK	NO_OS_BIT(6)
#define AD7746_EXCSETUP_EXCB_MSK	NO_OS_GENMASK(5,4)
#define AD7746_EXCSETUP_EXCA_MSK	NO_OS_GENMASK(3,2)
#define AD7746_EXCSETUP_EXCLVL_MSK	NO_OS_GENMASK(1,0)

/* AD7746_REG_CFG bits */
#define AD7746_CONF_VTF_MSK		NO_OS_GENMASK(7,6)
#define AD7746_CONF_CAPF_MSK		NO_OS_GENMASK(5,3)
#define AD7746_CONF_MD_MSK		NO_OS_GENMASK(2,0)

/* AD7746_REG_CAPDACx bits */
#define AD7746_CAPDAC_DACEN_MSK		NO_OS_BIT(7)
#define AD7746_CAPDAC_DACP_MSK		NO_OS_GENMASK(6,0)

//helper functions
void writeRegister(byte subaddress, byte value){
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(subaddress);
  Wire.write(value); 
  Wire.endTransmission();
}

byte readRegister(byte subaddress){
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(subaddress);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 1);
  if(Wire.availabe()) return Wire.read();
  return 0; 
}

bool dataReady(){
  //its in the status register to check if data ready
  byte status = readRegister(AD7746_REG_STATUS)
  // RDYCAP = 0 means data ready
  return (status & AD7746_STATUS_RDYCAP_MSK) == 0;
  return ready;
}

long readCapacitanceRaw(){
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_REG_CAP_DATA_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS,3); 
  if (Wire.available()<3) return -1//error
  long raw = ((long)Wire.read()<<16) | ((long)Wire.read()<<8) | ((long)Wire.read());
  return raw & 0xFFFFFF;//24-bit
}
void setup(){
  Serial.begin(115200) 
  Wire.begin(); // SDA=21, SCL=22 default on ESP32
  //enable i2c
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_RESET_CMD);
  Wire.endTransmission();
  delay(10);
  // Enable capacitance channel (mode)
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  //enable continuous conversion
  writeRegister(AD7746_REG_CFG,0xA0|0b00000001);
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
