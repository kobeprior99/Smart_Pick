#ifndef ACC_HPP
#define ACC_HPP

#include <Arduino.h>
#include <Wire.h>

// ## Accelerometer (I2C Interface)
// | Signal      | GPIO Pin |
// |-------------|----------|
// | SDA         | IO41     |
// | SCL         | IO40     |
// | Interrupt 1 | IO36     |
// | Interrupt 2 | IO35     |

// I2C address
#define ADXL375_ADDR 0x53

// Registers
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_BW_RATE     0x2C
#define REG_INT_ENABLE  0x2E
#define REG_INT_MAP     0x2F
#define REG_INT_SOURCE  0x30
#define REG_DATAX0      0x32
#define REG_SHX_THRSH   0x1D
#define REG_SHX_DUR     0x21
#define REG_DEVID       0x00
#define ADXL375_DEVID   0xE5
#define REG_ACT_THRSH   0x24
#define REG_ACT_INACT_CTL 0x27
// Pins
#define ACC_INT1_PIN 36
#define ACC_INT2_PIN 35
class ACC {
public:

    ACC(TwoWire &wire) : _wire(wire) {}

    bool Setup() {
      //set both interrupt pinmodes to input
      pinMode(ACC_INT1_PIN, INPUT);
      pinMode(ACC_INT2_PIN, INPUT);
      //verify i2c comms before writting any config
      uint8_t devid = readRegister(REG_DEVID);
      if (devid != ADXL375_DEVID){
        return false;
      }
      // Data rate
      writeRegister(REG_BW_RATE, 0x0D);   // 800 Hz
      // Data format
      writeRegister(REG_DATA_FORMAT, 0x0B);   // full resolution
      //enable activity on xyz axes 
      writeRegister(REG_ACT_INACT_CTL, 0b01110000);
      // 780 mg per lsb
      writeRegister(REG_ACT_THRSH, 0x07); // adjust sensitivity activity must go above and below the threshold 6 LSBs because I recorded 30 ish baseline from steady state
                                          
      // Route to INT1 to activity interrupt INT2 to data ready
      writeRegister(REG_INT_MAP, 0b10000000);
      // Enable newData and activity interrupt
      writeRegister(REG_INT_ENABLE, 0b10010000);
      writeRegister(0x38, 0x00);  // FIFO_CTL — bypass mode, no FIFO
      // Enable measurement mode 00001000
      writeRegister(REG_POWER_CTL, 0x08);
      //clear any interrupts
      readRegister(REG_INT_SOURCE);
      readAccelMagnitude();
      return true;
    }

    void debugPrint() {
        Serial.println("--- ADXL375 Register Readback ---");
        Serial.print("DEVID       (0x00): 0x"); Serial.println(readRegister(0x00), HEX);
        Serial.print("BW_RATE     (0x2C): 0b"); Serial.println(readRegister(REG_BW_RATE),     BIN);
        Serial.print("DATA_FORMAT (0x31): 0b"); Serial.println(readRegister(REG_DATA_FORMAT),  BIN);
        Serial.print("THRESH_ACT(0x24):   "); Serial.println(readRegister(REG_ACT_THRSH));
        Serial.print("INT_MAP     (0x2F): 0b"); Serial.println(readRegister(REG_INT_MAP),      BIN);
        Serial.print("INT_ENABLE  (0x2E): 0b"); Serial.println(readRegister(REG_INT_ENABLE),   BIN);
        Serial.print("INT_SOURCE  (0x30): 0b"); Serial.println(readRegister(REG_INT_SOURCE),   BIN);
        Serial.print("POWER_CTL   (0x2D): 0b"); Serial.println(readRegister(REG_POWER_CTL),   BIN);
        Serial.println("---------------------------------");
    }
    void debugAccel() {
        uint8_t buffer[6];
        _wire.beginTransmission(ADXL375_ADDR);
        _wire.write(REG_DATAX0);
        _wire.endTransmission(false);
        _wire.requestFrom(ADXL375_ADDR, 6);
        for (int i = 0; i < 6; i++) buffer[i] = _wire.read();

        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];
        uint32_t mag = sqrt((int32_t)x*x + (int32_t)y*y + (int32_t)z*z);

        Serial.print("X:"); Serial.print(x);
        Serial.print(" Y:"); Serial.print(y);
        Serial.print(" Z:"); Serial.print(z);
        Serial.print(" MAG:"); Serial.print(mag);
        Serial.print(" THR:"); Serial.println(readRegister(REG_SHX_THRSH));
    }
    void clearInterrupts() {
        // Reading INT_SOURCE clears all latched interrupts and
        // allows the pins to return low until the next event
        readRegister(REG_INT_SOURCE);
    }
    uint32_t readAccelMagnitude() {

        uint8_t buffer[6];

        _wire.beginTransmission(ADXL375_ADDR);
        _wire.write(REG_DATAX0);
        _wire.endTransmission(false);

        _wire.requestFrom(ADXL375_ADDR, 6);

        for(int i=0;i<6;i++)
            buffer[i] = _wire.read();

        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];

        uint32_t mag = sqrt((int32_t)x*x + (int32_t)y*y + (int32_t)z*z);

        return mag;//raw LSB needs to be converted with 49 mg/LSB in post
    }

private:

    TwoWire &_wire;

    void writeRegister(uint8_t reg, uint8_t value) {

        _wire.beginTransmission(ADXL375_ADDR);
        _wire.write(reg);
        _wire.write(value);
        _wire.endTransmission();
    }

    uint8_t readRegister(uint8_t reg) {

        _wire.beginTransmission(ADXL375_ADDR);
        _wire.write(reg);
        _wire.endTransmission(false);

        _wire.requestFrom(ADXL375_ADDR, (uint8_t)1);
        return _wire.read();
    }
};

#endif
