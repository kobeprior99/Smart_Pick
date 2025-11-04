/*
  ==============================================================================
  Project: cap_and_accel_stream.ino
  Authors: Kobe Prior and Sophia Mimlitz
  Description:
  Stream raw accelerometer and capacitance data from ESP32 to computer.
  FFT and plotting handled on PC.
  ==============================================================================
*/

#include <Wire.h>
#include <MPU6050.h>

#define AD7746_ADDRESS 0x48
#define AD7746_REG_STATUS 0x00
#define AD7746_REG_CAP_DATA_HIGH 0x01
#define AD7746_REG_CAP_SETUP 0x07
#define AD7746_REG_CFG 0x0A
#define AD7746_REG_EXC_SETUP 0x09
#define AD7746_STATUS_RDYCAP_MSK (1 << 0)
#define AD7746_CAPSETUP_CAPEN_MSK (1 << 7)

MPU6050 mpu;
unsigned long lastCapTime = 0;
unsigned long lastAccelTime = 0;

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 1);
  return Wire.available() ? Wire.read() : 0;
}

bool capDataReady() {
  uint8_t status = readRegister(AD7746_REG_STATUS);
  return (status & AD7746_STATUS_RDYCAP_MSK) == 0;
}

long readCapacitanceRaw() {
  Wire.beginTransmission(AD7746_ADDRESS);
  Wire.write(AD7746_REG_CAP_DATA_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AD7746_ADDRESS, 3);
  if (Wire.available() < 3) return -1;
  long raw = ((long)Wire.read() << 16) | ((long)Wire.read() << 8) | Wire.read();
  return raw & 0xFFFFFF;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Configure AD7746
  writeRegister(AD7746_REG_CAP_SETUP, AD7746_CAPSETUP_CAPEN_MSK);
  writeRegister(AD7746_REG_CFG, 0b10000001);
  writeRegister(AD7746_REG_EXC_SETUP, 0x03 | 0b00001000);

  // Configure MPU6050
  mpu.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Disable DLPF
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x19);  // Set sample rate divider
  Wire.write(4);     // 8kHz/(1+4)=1600Hz
  Wire.endTransmission();

Serial.println("time_ms,cap_raw,acc_x,acc_y,acc_z");
}

void loop() {
  unsigned long now = millis();

  // --- Capacitance Sampling (~90 SPS) ---
  if (capDataReady() && now - lastCapTime >= 10) {
    long capRaw = readCapacitanceRaw();
    lastCapTime = now;
    Serial.print("CAP")
    Serial.print(",");
    Serial.print(now);
    Serial.print(",");
    Serial.print(capRaw);
    Serial.print(",");
    Serial.print(mpu.getAccX(), 4);
    Serial.print(",");
    Serial.print(mpu.getAccY(), 4);
    Serial.print(",");
    Serial.println(mpu.getAccZ(), 4);
  }

  // --- Accelerometer Sampling (~1 kHz) ---
  if (now - lastAccelTime >= 1) {
    lastAccelTime = now;
    mpu.update();
    Serial.print("ACC");
    Serial.print(",");
    Serial.print(now);
    Serial.print(",");
    Serial.print(mpu.getAccX(), 4);
    Serial.print(",");
    Serial.print(mpu.getAccY(), 4);
    Serial.print(",");
    Serial.println(mpu.getAccZ(), 4);
    //we might be able to get away with only sending one direction
  }
  
}
