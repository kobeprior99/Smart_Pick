/*
  ==============================================================================
  Project:  Capacitance and Acceleration Sampling using ESP32
  File:     cap+accel.ino
  Authors:  Kobe Prior and Sophia Mimlitz
  Date:     November 6, 2025

  Description:
    This program samples both capacitance and acceleration values using an
    ESP32 microcontroller interfaced with:
      - CN0552-PMDZ Capacitance-to-Digital Converter (AD7746)
      - MPU6050 3-Axis Accelerometer and Gyroscope

    The ESP32 communicates with both devices over a shared I²C bus and 
    transmits the measured data over Serial for real-time monitoring and
    analysis in Python.

    The AD7746 provides high-resolution capacitance readings at ~90 samples 
    per second, while the MPU6050 captures vibration data in the x-axis at 
    approximately 1 kHz. These readings are tagged and timestamped for 
    near-simultaneous acquisition and time alignment in post-processing.

  Hardware Setup:
    SDA (CN0552 + MPU6050) -> GPIO 21 (ESP32)
    SCL (CN0552 + MPU6050) -> GPIO 22 (ESP32)
    Add 4.7kΩ pull-up resistors to SDA and SCL lines

    CN0552 (AD7746)
      - ±4.096 pF measurement range at maximum bulk capacitance of 17 pF
      - Extended mode: ±50 pF range, 200 pF bulk
      - Resolution: 4 aF
      - Accuracy: 4 fF
      - Continuous conversion mode at ~90 SPS

    MPU6050
      - Configured for high-rate acceleration sampling (~1 kHz)
      - x-axis data printed for vibration monitoring

  Serial Output Format:
    TIME,<seconds>,CAP,<raw_value>
    TIME,<seconds>,ACCEL,<ax_raw_value>

  Dependencies:
    - Wire.h (I²C communication)
    - MPU6050.h (Jeff Rowberg library)

  ==============================================================================
*/
#include <Wire.h>
#include "MPU6050.h"
// ==== AD7746 REGISTER AND HELPER DEFINITIONS ====
#define NO_OS_BIT(x) (1U << (x))
#define NO_OS_GENMASK(h, l) (((0xFF << (l)) & (0xFF >> (7 - (h)))))
#define AD7746_ADDRESS 0x48
#define AD7746_RESET_CMD 0xBF
#define AD7746_REG_STATUS 0u
#define AD7746_REG_CAP_DATA_HIGH 1u
#define AD7746_REG_CAP_DATA_MID  2u
#define AD7746_REG_CAP_DATA_LOW  3u
#define AD7746_REG_CAP_SETUP 7u
#define AD7746_REG_EXC_SETUP 9u
#define AD7746_REG_CFG 10u
#define AD7746_STATUS_RDYCAP_MSK NO_OS_BIT(0)
#define AD7746_CAPSETUP_CAPEN_MSK NO_OS_BIT(7)

//initialize accelerometer
MPU6050 mpu;

// ---- AD7746 Helper Functions ----
void writeRegister(uint8_t subaddress, uint8_t value) {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(subaddress);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t subaddress) {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(subaddress);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

bool dataReady() {
  uint8_t status = readRegister(AD7746_REG_STATUS);
  return (status & AD7746_STATUS_RDYCAP_MSK) == 0;
}

long readCapacitanceRaw() {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_REG_CAP_DATA_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 3);
  if (Wire.available() < 3) return -1;
  long raw = ((long)Wire.read() << 16) | ((long)Wire.read() << 8) | ((long)Wire.read());
  return raw & 0xFFFFFF;
}

// ==== TASKS ====
// Capacitance sampling task (~90 Hz)
void capTask(void *parameter) {
  for (;;) {
    if (dataReady()) {
      long raw = readCapacitanceRaw();
      float timestamp = millis() / 1000.0;
      Serial.print("TIME,");
      Serial.print(timestamp, 4);
      Serial.print(",CAP,");
      Serial.println(raw);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); // ~200 Hz polling
  }
}

// Accelerometer sampling task (~1 kHz)
void accelTask(void *parameter) {
  for (;;) {
    int16_t ax = mpu.getAccelerationX();  // Read only x-axis
    float timestamp = millis() / 1000.0;
    Serial.print("TIME,");
    Serial.print(timestamp, 4);
    Serial.print(",ACCEL,");
    Serial.println(ax);
    vTaskDelay(1 / portTICK_PERIOD_MS); // ~1 kHz
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // ==== AD7746 INIT ====
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_RESET_CMD);
  Wire.endTransmission();
  delay(10);
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  writeRegister(AD7746_REG_CFG, 0b10000001); // continuous conversion, 90 SPS
  writeRegister(AD7746_REG_EXC_SETUP, 0x03 | 0b00001000);

  // ==== MPU6050 INIT ====
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 ready");
  }

  // Start tasks on separate cores
  xTaskCreatePinnedToCore(capTask, "capTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(accelTask, "accelTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Tasks handle all data collection and serial transmission
}
