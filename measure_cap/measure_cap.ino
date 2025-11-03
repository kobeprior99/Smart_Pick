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

//wire library for i2c
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
void writeRegister(uint8_t subaddress, uint8_t value){
  //start transmission with CDC
  Wire.beginTransmission(AD7746_ADDRESS);
  //set address pointer register to subaddress
  Wire.write(subaddress);
  //write a byte to the address pointed to above
  Wire.write(value); 
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t subaddress){
  //start transmission with CDC
  Wire.beginTransmission(AD7746_ADDRESS);
  //set address pointer register to subaddress
  Wire.write(subaddress);
  Wire.endTransmission(false);
  //read from the address pointed to from above
  Wire.requestFrom(AD7746_ADDRESS, 1);
  if(Wire.available()) return Wire.read();
  return 0; 
}

bool dataReady(){
  //get RDY bit from status register 
  uint8_t status = readRegister(AD7746_REG_STATUS);

  // RDYCAP = 0 means data ready
  return (status & AD7746_STATUS_RDYCAP_MSK) == 0b00000000;
}

long readCapacitanceRaw(){
  //the device address
  Wire.beginTransmission(AD7746_ADDRESS);
  //register address pointer
  Wire.write(AD7746_REG_CAP_DATA_HIGH);
  Wire.endTransmission(false);
  //read 3 bytes and stich them together, address pointer auto incrememnts
  Wire.requestFrom(AD7746_ADDRESS,3); 
  if (Wire.available()<3) return -1; //error
  long raw = ((long)Wire.read()<<16) | ((long)Wire.read()<<8) | ((long)Wire.read());
  return raw & 0xFFFFFF;//24-bit
}

void setup(){
  //start serial
  Serial.begin(115200);
  //start i2c
  Wire.begin(); // SDA=21, SCL=22 default on ESP32

  //reset the CDC
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_RESET_CMD);
  Wire.endTransmission();
  delay(10);

  // Enable capacitance channel (mode)
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  //enable continuous conversion and faster sampling rate
  writeRegister(AD7746_REG_CFG,0b10000001);
  //enable excitation
  writeRegister(AD7746_REG_EXC_SETUP, 0x03|0b00001000);
  //enable 90 sps  
  writeRegister()
}

void loop() {
  //check if the user typed something 
  if (Serial.available() > 0){
    char input = Serial.read(); //read one character
    if (input == 'q'|| input == 'Q'){
      Serial.println("Exiting loop...");
      while(true){
        if (Serial.available() > 0){
          char input = Serial.read(); //read one character
          if (input == 'r'|| input == 'R'){
            Serial.println("Entering loop...");
            break;
          }
        }
      }
      //stop running until another input is entered
    }
  }
  //check if data ready then read capacitance 
  if (dataReady()) {
    long raw = readCapacitanceRaw();
    //print capacitance to serial monitor
    Serial.print("Capacitance Raw: ");
    Serial.print(raw);
    Serial.println(" ");
  }
}
