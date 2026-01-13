#ifndef I2C_HELPER
#define I2C_HELPER
//functions to generate bit masks
#define NO_OS_BIT(x) (1U << (x))
#define NO_OS_GENMASK(h, l) (((0xFF << (l)) & (0xFF >> (7 - (h)))))
//define registers from the driver file:
/* AD7746 Slave Address */
#define AD7746_ADDRESS 0x48

/* AD7746 Reset command */
#define AD7746_RESET_CMD 0xBF

/* AD7746 Register Definition */
#define AD7746_REG_STATUS 0u
#define AD7746_REG_CAP_DATA_HIGH 1u
#define AD7746_REG_CAP_DATA_MID 2u
#define AD7746_REG_CAP_DATA_LOW 3u
#define AD7746_REG_VT_DATA_HIGH 4u
#define AD7746_REG_VT_DATA_MID 5u
#define AD7746_REG_VT_DATA_LOW 6u
#define AD7746_REG_CAP_SETUP 7u
#define AD7746_REG_VT_SETUP 8u
#define AD7746_REG_EXC_SETUP 9u
#define AD7746_REG_CFG 10u
#define AD7746_REG_CAPDACA 11u
#define AD7746_REG_CAPDACB 12u
#define AD7746_REG_CAP_OFFH 13u
#define AD7746_REG_CAP_OFFL 14u
#define AD7746_REG_CAP_GAINH 15u
#define AD7746_REG_CAP_GAINL 16u
#define AD7746_REG_VOLT_GAINH 17u
#define AD7746_REG_VOLT_GAINL 18u

#define AD7746_NUM_REGISTERS (AD7746_REG_VOLT_GAINL + 1u)

/* AD7746_REG_STATUS bits */
#define AD7746_STATUS_EXCERR_MSK NO_OS_BIT(3)
#define AD7746_STATUS_RDY_MSK NO_OS_BIT(2)
#define AD7746_STATUS_RDYVT_MSK NO_OS_BIT(1)
#define AD7746_STATUS_RDYCAP_MSK NO_OS_BIT(0)

/* AD7746_REG_CAP_SETUP bits */
#define AD7746_CAPSETUP_CAPEN_MSK NO_OS_BIT(7)
#define AD7746_CAPSETUP_CIN2_MSK NO_OS_BIT(6)
#define AD7746_CAPSETUP_CAPDIFF_MSK NO_OS_BIT(5)
#define AD7746_CAPSETUP_CAPCHOP_MSK NO_OS_BIT(0)

/* AD7746_REG_VT_SETUP bits */
#define AD7746_VTSETUP_VTEN_MSK NO_OS_BIT(7)
#define AD7746_VTSETUP_VTMD_MSK NO_OS_GENMASK(6, 5)
#define AD7746_VTSETUP_EXTREF_MSK NO_OS_BIT(4)
#define AD7746_VTSETUP_VTSHORT_MSK NO_OS_BIT(1)
#define AD7746_VTSETUP_VTCHOP_MSK NO_OS_BIT(0)

/* AD7746_REG_EXC_SETUP bits */
#define AD7746_EXCSETUP_CLKCTRL_MSK NO_OS_BIT(7)
#define AD7746_EXCSETUP_EXCON_MSK NO_OS_BIT(6)
#define AD7746_EXCSETUP_EXCB_MSK NO_OS_GENMASK(5, 4)
#define AD7746_EXCSETUP_EXCA_MSK NO_OS_GENMASK(3, 2)
#define AD7746_EXCSETUP_EXCLVL_MSK NO_OS_GENMASK(1, 0)

/* AD7746_REG_CFG bits */
#define AD7746_CONF_VTF_MSK NO_OS_GENMASK(7, 6)
#define AD7746_CONF_CAPF_MSK NO_OS_GENMASK(5, 3)
#define AD7746_CONF_MD_MSK NO_OS_GENMASK(2, 0)

/* AD7746_REG_CAPDACx bits */
#define AD7746_CAPDAC_DACEN_MSK NO_OS_BIT(7)
#define AD7746_CAPDAC_DACP_MSK NO_OS_GENMASK(6, 0)
class fclass{
  public:
    fclass();    
    //helper functions
    void Setup();
    void writeRegister(uint8_t subaddress, uint8_t value);
    uint8_t readRegister(uint8_t subaddress);
    bool dataReady();
    long readCapacitanceRaw(); 
};
extern fclass cdc;
#endif

