#pragma once
#include <Arduino.h>
#include <SPI.h>

class FlashMemory {
public:
    void begin();

    bool writeBytes(uint32_t address, uint8_t* data, size_t length);
    bool readBytes(uint32_t address, uint8_t* buffer, size_t length);

    void eraseSector(uint32_t address);

private:
    void select();
    void deselect();
    void writeEnable();
};
