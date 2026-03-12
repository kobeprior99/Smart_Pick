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

// Pins
#define ACC_INT1_PIN 36
class ACC {
public:

    ACC(TwoWire &wire) : _wire(wire) {}

    void Setup() {
      pinMode(ACC_INT1_PIN, INPUT);
      // Data rate
      writeRegister(REG_BW_RATE, 0x0D);   // 800 Hz
      // Data format
      writeRegister(REG_DATA_FORMAT, 0x0B);   // full resolution
      // Shock threshold
      writeRegister(REG_SHX_THRSH, 40);     // adjust sensitivity 2.5 g
      // Shock duration 625us per LSB
      writeRegister(REG_SHX_DUR, 0x01);
      // Route to INT1 to pin Int pin  1
      writeRegister(REG_INT_MAP, 0x00);
      // Enable single shock interrupt
      writeRegister(REG_INT_ENABLE, 0b01000000);
      // Enable measurement mode 00001000
      writeRegister(REG_POWER_CTL, 0x08);
    }

    bool Interrupt1() {
        //if this pin goes high we know shock interrupt happened
        if (digitalRead(ACC_INT1_PIN)) {

            // clear interrupt by reading source register
            readRegister(REG_INT_SOURCE);

            return true;
        }

        return false;
    }

    int16_t readAccelMagnitude() {

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

        int32_t mag = sqrt((int32_t)x*x + (int32_t)y*y + (int32_t)z*z);

        return (int16_t)mag;
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
