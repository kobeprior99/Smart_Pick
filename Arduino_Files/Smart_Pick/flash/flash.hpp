/*
 * EDNS 492 - Senior Design
 * Smart Pick for Rock Mining
 * Sophia Mimlitz
 * 2/26/2026
 * Description: Header file for Flash Memory programming
 */

#ifndef FLASH_H
#define FLASH_H

#include <Arduino.h>
#include <SPI.h>

// USER CONFIGURATION

// SPI pins
// ## Flash Memory (SPI Interface)
// | Signal | GPIO Pin |
// |--------|----------|
// | SCK    | IO47     |
// | MOSI   | IO48     |
// | MISO   | IO13     |
// | CS     | IO14     |
#define FLASH_PIN_CS   14
#define FLASH_PIN_MOSI 48
#define FLASH_PIN_MISO 13
#define FLASH_PIN_SCK  47 

// Flash geometry
#define FLASH_PAGE_SIZE    256
#define FLASH_SECTOR_SIZE  4096

// Flash command set
#define FLASH_CMD_WREN   0x06
#define FLASH_CMD_RDSR1  0x05
#define FLASH_CMD_SE_4K  0x20
#define FLASH_CMD_PP     0x02
#define FLASH_CMD_READ   0x03
#define FLASH_CMD_RDID   0x9F

#define FLASH_SR1_WIP    0x01

// PACKET FORMAT
struct __attribute__((packed)) FlashPacket {
    uint32_t timestamp;
    uint32_t mag_accel;
    uint32_t capacitance;
};
// Total = 12 bytes = 4 + 4 + 4
// FLASH LOGGER CLASS

template <size_t PACKETS_PER_BUFFER>
class flash {
public:
    flash()
        : writePtr(0),
          logBaseAddr(0),
          logSizeBytes(0),
          fillingRed(true),
          fillIndex(0),
          redReady(false),
          blueReady(false),
          overflowed(false),
          packetsLogged(0) {}

    // SPI / Device Init
    bool begin(uint32_t spiFrequency = 10000000) {
        pinMode(FLASH_PIN_CS, OUTPUT);
        digitalWrite(FLASH_PIN_CS, HIGH);

        SPI.begin(FLASH_PIN_SCK, FLASH_PIN_MISO, FLASH_PIN_MOSI);
        SPI.beginTransaction(SPISettings(spiFrequency, MSBFIRST, SPI_MODE0));

        uint8_t id[3];
        return readJedecID(id);
    }

    // Log Region Setup
    void configureLogRegion(uint32_t baseAddr, uint32_t sizeBytes) {
        logBaseAddr = baseAddr;
        logSizeBytes = sizeBytes;
        writePtr = logBaseAddr;

        fillingRed = true;
        fillIndex = 0;
        redReady = false;
        blueReady = false;
        overflowed = false;
        packetsLogged = 0;
    }

    bool eraseLogRegion() {
        if (logSizeBytes == 0) return false;

        for (uint32_t addr = logBaseAddr;
             addr < logBaseAddr + logSizeBytes;
             addr += FLASH_SECTOR_SIZE) {
            if (!eraseSector4K(addr)) return false;
        }

        writePtr = logBaseAddr;
        return true;
    }

    // Packet Logging
    bool appendPacket(const FlashPacket& packet) {
        if (overflowed) return false;

        if (fillingRed) {
            redBuffer[fillIndex] = packet;
        } else {
            blueBuffer[fillIndex] = packet;
        }

        fillIndex++;
        packetsLogged++;

        if (fillIndex >= PACKETS_PER_BUFFER) {
            if (fillingRed) {
                if (redReady) {
                    overflowed = true;
                    return false;
                }
                redReady = true;
                fillingRed = false;
            } else {
                if (blueReady) {
                    overflowed = true;
                    return false;
                }
                blueReady = true;
                fillingRed = true;
            }

            fillIndex = 0;
        }

        return true;
    }

    // Call this often in loop() to push any full buffer to flash
    bool serviceFlashWrite() {
        if (redReady) {
            if (!writeBufferToFlash((const uint8_t*)redBuffer, sizeof(redBuffer))) {
                return false;
            }
            redReady = false;
            return true;
        }

        if (blueReady) {
            if (!writeBufferToFlash((const uint8_t*)blueBuffer, sizeof(blueBuffer))) {
                return false;
            }
            blueReady = false;
            return true;
        }

        return true;
    }

    // Flush the partially filled active buffer at the end of experiment
    bool finalizeLog() {
        if (!serviceFlashWrite()) return false;
        if (!serviceFlashWrite()) return false;

        if (fillIndex == 0) return true;

        size_t partialBytes = fillIndex * sizeof(FlashPacket);

        if (fillingRed) {
            if (!writeBufferToFlash((const uint8_t*)redBuffer, partialBytes)) {
                return false;
            }
        } else {
            if (!writeBufferToFlash((const uint8_t*)blueBuffer, partialBytes)) {
                return false;
            }
        }

        fillIndex = 0;
        return true;
    }

    // Status
    bool hasOverflowed() const {
        return overflowed;
    }

    bool hasPendingBuffer() const {
        return redReady || blueReady || (fillIndex > 0);
    }

    uint32_t getWritePointer() const {
        return writePtr;
    }

    uint32_t getPacketsLogged() const {
        return packetsLogged;
    }

    size_t getPacketSize() const {
        return sizeof(FlashPacket);
    }

    // Read Back
    void read(uint32_t addr, uint8_t* buffer, size_t len) {
        select();
        SPI.transfer(FLASH_CMD_READ);
        sendAddress(addr);

        for (size_t i = 0; i < len; i++) {
            buffer[i] = SPI.transfer(0x00);
        }

        deselect();
    }

private:
    // Internal State
    uint32_t writePtr;
    uint32_t logBaseAddr;
    uint32_t logSizeBytes;

    FlashPacket redBuffer[PACKETS_PER_BUFFER];
    FlashPacket blueBuffer[PACKETS_PER_BUFFER];

    bool fillingRed;
    size_t fillIndex;

    bool redReady;
    bool blueReady;
    bool overflowed;

    uint32_t packetsLogged;

    // Low-Level Flash Helpers
    void select() {
        digitalWrite(FLASH_PIN_CS, LOW);
    }

    void deselect() {
        digitalWrite(FLASH_PIN_CS, HIGH);
    }

    void sendAddress(uint32_t addr) {
        SPI.transfer((addr >> 16) & 0xFF);
        SPI.transfer((addr >> 8) & 0xFF);
        SPI.transfer(addr & 0xFF);
    }

    void writeEnable() {
        select();
        SPI.transfer(FLASH_CMD_WREN);
        deselect();
    }

    uint8_t readStatus1() {
        select();
        SPI.transfer(FLASH_CMD_RDSR1);
        uint8_t sr1 = SPI.transfer(0x00);
        deselect();
        return sr1;
    }

    bool waitUntilReady(uint32_t timeout_ms) {
        uint32_t start = millis();
        while (readStatus1() & FLASH_SR1_WIP) {
            if (millis() - start > timeout_ms) {
                return false;
            }
            delay(1);
        }
        return true;
    }

    bool readJedecID(uint8_t id[3]) {
        select();
        SPI.transfer(FLASH_CMD_RDID);
        id[0] = SPI.transfer(0x00);
        id[1] = SPI.transfer(0x00);
        id[2] = SPI.transfer(0x00);
        deselect();

        if ((id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00) ||
            (id[0] == 0xFF && id[1] == 0xFF && id[2] == 0xFF)) {
            return false;
        }

        return true;
    }

    bool eraseSector4K(uint32_t addr) {
        uint32_t sectorBase = addr & ~(FLASH_SECTOR_SIZE - 1);

        if (!inLogRegion(sectorBase, FLASH_SECTOR_SIZE)) {
            return false;
        }

        writeEnable();

        select();
        SPI.transfer(FLASH_CMD_SE_4K);
        sendAddress(sectorBase);
        deselect();

        return waitUntilReady(8000);
    }

    bool pageProgram(uint32_t addr, const uint8_t* data, size_t len) {
        if (len == 0 || len > FLASH_PAGE_SIZE) return false;

        uint32_t pageOffset = addr % FLASH_PAGE_SIZE;
        if (pageOffset + len > FLASH_PAGE_SIZE) return false;

        if (!inLogRegion(addr, len)) return false;

        writeEnable();

        select();
        SPI.transfer(FLASH_CMD_PP);
        sendAddress(addr);

        for (size_t i = 0; i < len; i++) {
            SPI.transfer(data[i]);
        }

        deselect();

        return waitUntilReady(2000);
    }

    bool appendRaw(const uint8_t* data, size_t len) {
        if (!inLogRegion(writePtr, len)) return false;

        while (len > 0) {
            uint32_t pageOffset = writePtr % FLASH_PAGE_SIZE;
            size_t spaceInPage = FLASH_PAGE_SIZE - pageOffset;
            size_t chunk = (len < spaceInPage) ? len : spaceInPage;

            if (!pageProgram(writePtr, data, chunk)) {
                return false;
            }

            writePtr += chunk;
            data += chunk;
            len -= chunk;
        }

        return true;
    }

    bool writeBufferToFlash(const uint8_t* buffer, size_t len) {
        return appendRaw(buffer, len);
    }

    bool inLogRegion(uint32_t addr, size_t len) const {
        if (addr < logBaseAddr) return false;

        uint64_t endAddr = (uint64_t)addr + (uint64_t)len;
        uint64_t logEnd  = (uint64_t)logBaseAddr + (uint64_t)logSizeBytes;

        return endAddr <= logEnd;
    }
};

#endif
