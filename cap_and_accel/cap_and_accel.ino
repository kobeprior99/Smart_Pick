//accelerometer library
#include <MPU6050.h>
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

TaskHandle_t capTaskHandle;
TaskHandle_t accelTaskHandle;




//---------------- Capacitance Task (Core 0) ----------------//
void capacitanceTask(void *parameter) {
  for (;;) {
    if (dataReady()) {
      long raw = readCapacitanceRaw();
      Serial.print("[Core 0] Capacitance Raw: ");
      Serial.println(raw);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // small delay to yield CPU
  }
}

//---------------- Accelerometer Task (Core 1) ----------------//
void accelTask(void *parameter) {
  for (;;) {
    mpu.update();
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    Serial.print("[Core 1] Accel (g): ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.println(az, 3);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//---------------- Setup ----------------//
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing...");

  // Initialize I2C buses
  Wire.begin(21, 22);   // Capacitance (AD7746)
  Wire1.begin(25, 26);  // Accelerometer (MPU6050)

  //---- AD7746 Configuration ----//
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_RESET_CMD);
  Wire.endTransmission();
  delay(10);
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  writeRegister(AD7746_REG_CFG, 0xA0 | 0b00000001);
  writeRegister(AD7746_REG_EXC_SETUP, 0x03 | 0b00001000);

  //---- MPU6050 Configuration ----//
  mpu.begin(Wire1);
  mpu.calcOffsets(); // optional calibration
  Serial.println("MPU6050 initialized!");

  // Create task for capacitance (Core 0)
  xTaskCreatePinnedToCore(
    capacitanceTask,
    "Capacitance Task",
    4096,
    NULL,
    1,
    &capTaskHandle,
    0 // Core 0
  );

  // Create task for accelerometer (Core 1)
  xTaskCreatePinnedToCore(
    accelTask,
    "Accelerometer Task",
    4096,
    NULL,
    1,
    &accelTaskHandle,
    1 // Core 1
  );

  Serial.println("Tasks started on separate cores!");
}

void loop() {
  // Leave empty; tasks handle everything
}
