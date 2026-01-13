#ifndef CDC_HPP
#define CDC_HPP 

#include <Arduino.h>
#include <Wire.h>

//---BIT HELPERS---
#define NO_OS_BIT(x) (1U << (x))
#define NO_OS_GENMASK(h, l) (((0xFF << (l)) & (0xFF >> (7 - (h)))))
//-- AD7746 Definitions
/* AD7746 Slave Address */
#define AD7746_ADDRESS 0x48

/* AD7746 Reset command */
#define AD7746_RESET_CMD 0xBF

/* AD7746 Register Definition */
#define AD7746_REG_STATUS          0u
#define AD7746_REG_CAP_DATA_HIGH   1u
#define AD7746_REG_CAP_DATA_MID    2u
#define AD7746_REG_CAP_DATA_LOW    3u
#define AD7746_REG_VT_DATA_HIGH    4u
#define AD7746_REG_VT_DATA_MID     5u
#define AD7746_REG_VT_DATA_LOW     6u
#define AD7746_REG_CAP_SETUP       7u
#define AD7746_REG_VT_SETUP        8u
#define AD7746_REG_EXC_SETUP       9u
#define AD7746_REG_CFG            10u
#define AD7746_REG_CAPDACA        11u
#define AD7746_REG_CAPDACB        12u
#define AD7746_REG_CAP_OFFH       13u
#define AD7746_REG_CAP_OFFL       14u
#define AD7746_REG_CAP_GAINH      15u
#define AD7746_REG_CAP_GAINL      16u
#define AD7746_REG_VOLT_GAINH     17u
#define AD7746_REG_VOLT_GAINL     18u

#define AD7746_NUM_REGISTERS (AD7746_REG_VOLT_GAINL + 1u)

/* STATUS bits */
#define AD7746_STATUS_EXCERR_MSK NO_OS_BIT(3)
#define AD7746_STATUS_RDY_MSK    NO_OS_BIT(2)
#define AD7746_STATUS_RDYVT_MSK  NO_OS_BIT(1)
#define AD7746_STATUS_RDYCAP_MSK NO_OS_BIT(0)

/* CAP SETUP bits */
#define AD7746_CAPSETUP_CAPEN_MSK     NO_OS_BIT(7)
#define AD7746_CAPSETUP_CIN2_MSK      NO_OS_BIT(6)
#define AD7746_CAPSETUP_CAPDIFF_MSK   NO_OS_BIT(5)
#define AD7746_CAPSETUP_CAPCHOP_MSK   NO_OS_BIT(0)

/* VT SETUP bits */
#define AD7746_VTSETUP_VTEN_MSK       NO_OS_BIT(7)
#define AD7746_VTSETUP_VTMD_MSK       NO_OS_GENMASK(6, 5)
#define AD7746_VTSETUP_EXTREF_MSK     NO_OS_BIT(4)
#define AD7746_VTSETUP_VTSHORT_MSK    NO_OS_BIT(1)
#define AD7746_VTSETUP_VTCHOP_MSK     NO_OS_BIT(0)

/* EXC SETUP bits */
#define AD7746_EXCSETUP_CLKCTRL_MSK   NO_OS_BIT(7)
#define AD7746_EXCSETUP_EXCON_MSK     NO_OS_BIT(6)
#define AD7746_EXCSETUP_EXCB_MSK      NO_OS_GENMASK(5, 4)
#define AD7746_EXCSETUP_EXCA_MSK      NO_OS_GENMASK(3, 2)
#define AD7746_EXCSETUP_EXCLVL_MSK    NO_OS_GENMASK(1, 0)

/* CFG bits */
#define AD7746_CONF_VTF_MSK           NO_OS_GENMASK(7, 6)
#define AD7746_CONF_CAPF_MSK          NO_OS_GENMASK(5, 3)
#define AD7746_CONF_MD_MSK            NO_OS_GENMASK(2, 0)

/* CAPDAC bits */
#define AD7746_CAPDAC_DACEN_MSK       NO_OS_BIT(7)
#define AD7746_CAPDAC_DACP_MSK        NO_OS_GENMASK(6, 0)
//-- cdc class
class CDC {
  public:
    void Setup() {                     
      //reset the CDC
      Wire.beginTransmission(AD7746_ADDRESS);
      Wire.write(AD7746_RESET_CMD);
      Wire.endTransmission();
      delay(10);

      // Enable capacitance channel
      writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);

      // Continuous conversion, faster sampling
      writeRegister(AD7746_REG_CFG, 0b10000001);

      // Enable excitation A & B, B inverted from A
      writeRegister(AD7746_REG_EXC_SETUP, 0x03 | 0b00011000);

      // Apply offset DAC
      //0xFF top of range
      writeRegister(AD7746_REG_CAPDACA, 0);
    }

    bool dataReady() {
      uint8_t status = readRegister(AD7746_REG_STATUS);
      return status & AD7746_STATUS_RDYCAP_MSK;
    }

    long readCapacitanceRaw() {
      uint32_t value = 0;

      value |= ((uint32_t)readRegister(AD7746_REG_CAP_DATA_HIGH)) << 16;
      value |= ((uint32_t)readRegister(AD7746_REG_CAP_DATA_MID)) << 8;
      value |= ((uint32_t)readRegister(AD7746_REG_CAP_DATA_LOW));

      return (long)value;
    }

  private:
    void writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(AD7746_ADDRESS);
      Wire.write(reg);
      Wire.write(value);
      Wire.endTransmission();
    }

    uint8_t readRegister(uint8_t reg) {
      Wire.beginTransmission(AD7746_ADDRESS);
      Wire.write(reg);
      Wire.endTransmission(false);  // repeated start
      Wire.requestFrom(AD7746_ADDRESS, (uint8_t)1);
      return Wire.read();
    }
};



#endif
