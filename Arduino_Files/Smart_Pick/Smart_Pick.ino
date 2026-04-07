/**
 * @file    Smart_Pick.ino
 * @brief   Smart Pick — activity-triggered capacitance & acceleration logger
 *
 * Records 3-axis acceleration magnitude and raw capacitance to an
 * on-board SPI flash chip (128 Mbit / 16 MB).  An accelerometer
 * activity interrupt (INT1) arms the recorder; a data-ready interrupt
 * (INT2) drives sample acquisition at ~800 Hz.  A zero-order-hold
 * strategy carries the last valid CDC reading forward between
 * conversions so every flash packet has both sensor values.
 *
 * Serial commands (115200 baud):
 *   R  Read all logged packets back as CSV (timestamp_us, accel, cap)
 *   E  End recording and flush the write buffer to flash
 *   D  Bulk-erase the entire log region (~2 min)
 *   T  Write and verify a 10-packet test pattern
 *   I  Increment the CAPDAC offset by N codes (e.g. "I5")
 *
 * Hardware connections:
 *   I2C bus 0 — CDC converter   SDA=11  SCL=10  (400 kHz)
 *   I2C bus 1 — Accelerometer   SDA=41  SCL=40  (400 kHz)
 *   SPI       — Flash chip      CS/CLK/MOSI/MISO (device-defined pins)
 *   INT1      — Activity ISR    (ACC_INT1_PIN, RISING)
 *   INT2      — Data-ready ISR  (ACC_INT2_PIN, RISING)
 *
 * Dependencies:
 *   flash/flash.hpp          Double-buffered SPI flash logger
 *   accel/Accelerometer.hpp  Accelerometer driver (magnitude output)
 *   cdc/CDCConverter.hpp     Capacitance-to-digital converter driver
 *   leds/LEDs.hpp            Status LED state machine
 *   Wire.h                   Arduino TwoWire (ESP32)
 *
 * @author  Kobe Prior
 * @date    April 7 2026
 * @version 11.1.2
 */
#include "flash/flash.hpp"
#include "accel/Accelerometer.hpp"
#include "cdc/CDCConverter.hpp"
#include "leds/LEDs.hpp"

#include <Wire.h>

//objects

TwoWire I2C_CDC = TwoWire(0);  // SDA=11, SCL=10
TwoWire I2C_ACC = TwoWire(1);  // SDA=41, SCL=40

// ── Peripheral objects ────────────────────────────────────────────────────────
LEDs leds;
CDC cdc(I2C_CDC);
ACC acc(I2C_ACC);

// ── Flash logger ──────────────────────────────────────────────────────────────
// 64 packets per buffer — at 800 Hz that gives 80 ms of headroom per buffer,
// which comfortably covers the worst-case SPI sector-erase time (~50 ms).
flash<64> flashLogger;
#define LOG_BASE_ADDR 0x000000
#define LOG_SIZE_BYTES 0x1000000  // 16 MB — full 128 Mbit chip

// --ISRs _________________________________
volatile bool recording = false;
volatile bool dataReady = false;
volatile uint32_t start_time = 0;
volatile uint32_t isrTimestamp = 0;
uint32_t zohCapacitance = 0;

void IRAM_ATTR onActivityISR() {
  if (!recording) {
    start_time = micros();
    recording = true;
  }
}

void IRAM_ATTR onDataReadyISR() {
  dataReady = true;  //just set the flag so the loop gets the newest value
  isrTimestamp = micros();
}


//serial handler
void checkSerialCommand() {
  char cmd = Serial.read();

  // 'R' — read all logged packets back over serial as CSV
  if (cmd == 'R' || cmd == 'r') {
    leds.Read_Data();
    recording = false;

    flashLogger.finalizeLog();

    uint32_t totalBytes = flashLogger.getPacketsLogged() * flashLogger.getPacketSize();
    Serial.println("timestamp_us,acceleration,capacitance");

    FlashPacket fp;
    uint32_t packetSize = flashLogger.getPacketSize();
    for (uint32_t addr = LOG_BASE_ADDR;
         addr < LOG_BASE_ADDR + totalBytes;
         addr += packetSize) {
      flashLogger.read(addr, (uint8_t*)&fp, sizeof(FlashPacket));
      Serial.print(fp.timestamp);
      Serial.print(",");
      Serial.print(fp.mag_accel);
      Serial.print(",");
      Serial.println(fp.capacitance);
    }
    Serial.println("EOF");
    leds.End_Read_Data();
  }

  // 'E' — end recording, flush to flash
  if (cmd == 'E' || cmd == 'e') {
    recording = false;
    leds.End_Recording();
    flashLogger.finalizeLog();
    Serial.print("Recording ended. Packets logged: ");
    Serial.println(flashLogger.getPacketsLogged());
  }

  // 'D' — erase flash
  if (cmd == 'D' || cmd == 'd') {
    recording = false;
    leds.Read_Data();
    Serial.println("Erasing flash...this takes about 2 minutes, please wait...");
    flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
    flashLogger.eraseLogRegion();
    Serial.println("Flash erased.");
    leds.End_Read_Data();
  }
  // 'T' — write known test pattern and read back to verify flash integrity
  if (cmd == 'T' || cmd == 't') {
    recording = false;
    Serial.println("Writing test pattern to flash...");
    flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
    flashLogger.eraseSectorForTest();

    // Write 10 packets with known, easily recognizable values
    for (uint32_t i = 0; i < 10; i++) {
      FlashPacket pkt;
      pkt.timestamp = i * 1000;   // 0, 1000, 2000, 3000...
      pkt.mag_accel = i * 111;    // 0, 111, 222, 333...
      pkt.capacitance = i * 999;  // 0, 999, 1998, 2997...
      pkt.reserved = 0;
      flashLogger.appendPacket(pkt);
    }
    // Force everything to flash
    flashLogger.finalizeLog();

    // Read back and verify
    Serial.println("Reading back test pattern:");
    Serial.println("expected_ts, got_ts, expected_mag, got_mag, expected_cap, got_cap, pass");

    bool allPassed = true;
    uint32_t packetSize = flashLogger.getPacketSize();

    for (uint32_t i = 0; i < 10; i++) {
      FlashPacket fp;
      flashLogger.read(LOG_BASE_ADDR + i * packetSize, (uint8_t*)&fp, sizeof(FlashPacket));

      uint32_t expectedTs = i * 1000;
      uint32_t expectedMag = i * 111;
      uint32_t expectedCap = i * 999;

      bool pass = (fp.timestamp == expectedTs) && (fp.mag_accel == expectedMag) && (fp.capacitance == expectedCap);

      if (!pass) allPassed = false;

      Serial.print(expectedTs);
      Serial.print(",");
      Serial.print(fp.timestamp);
      Serial.print(",");
      Serial.print(expectedMag);
      Serial.print(",");
      Serial.print(fp.mag_accel);
      Serial.print(",");
      Serial.print(expectedCap);
      Serial.print(",");
      Serial.print(fp.capacitance);
      Serial.print(",");
      Serial.println(pass ? "PASS" : "FAIL");
    }

    Serial.println(allPassed ? "ALL PASSED" : "FAILURES DETECTED");
  }
  if (cmd == 'I' || cmd == 'i') {
    // Wait until the user sends something
    int amount = Serial.parseInt();
    cdc.incrementCapDAC(amount);
    // Flush two full conversions by actually reading the data registers
    for (int i = 0; i < 2; i++) {
        while (!cdc.dataReady()) { delay(1); }
        cdc.readCapacitanceRaw();  // this clears RDYCAP by reading the data registers
    }
    uint32_t raw = cdc.readCapacitanceRaw();
    leds.INC_CAPDAC();//adds 100ms delay
    Serial.print("0x");
    Serial.print(cdc.getCapDAC(), HEX);
    Serial.print(",");
    Serial.println(raw);
  }
}

//setup
void setup() {
  leds.Init();
  Serial.begin(115200);
  I2C_CDC.begin(11, 10, 400000);
  I2C_ACC.begin(41, 40, 400000);
  if (!cdc.Setup()) Serial.println("ERROR: CDC not found");
  else {
    Serial.println("CDC ok"); 
    leds.I2C_Passed();
  }


  if (!acc.Setup()) Serial.println("ERROR: ACC not found");
  else {
    Serial.println("ACC ok"); 
    leds.I2C_Passed();
  }

  if (!flashLogger.begin()) {
    Serial.println("ERROR: Flash not found");
    while (true) {
      leds.ThresholdA_Reached();
      delay(500);
    }
  }
  flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
  // Serial.print("Page write time (us): ");
  // Serial.println(flashLogger.measurePageWriteTime());// 176 us fast enough to service during sample downtime

  // Serial.print("Sector erase time(us): ");
  // Serial.println(flashLogger.measureSectorEraseTime());//32ms, kinda takes a while...

  attachInterrupt(digitalPinToInterrupt(ACC_INT1_PIN), onActivityISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ACC_INT2_PIN), onDataReadyISR, RISING);

  Serial.println("Smart Pick ready. Waiting for activity to begin recording...");
}

//loop---
void loop() {
 // only do this to measure capacitance during trial
  // Serial.print("Min:");
  // Serial.print(0);
  // Serial.print(",");
  // Serial.print("[raw-24bit]capacitance:");
  // Serial.print(zohCapacitance);
  // Serial.print(",");
  // Serial.print("Max:");
  // Serial.println(16777215);

  if (Serial.available()) {
    checkSerialCommand();
    return;
  }

  static bool ledSet = false;
  if (recording && !ledSet) {
    ledSet = true;
    leds.Start_Recording();
    Serial.println("Activity detected - recording started");
    //make sure data ready interrupt pin goes low
    while (digitalRead(ACC_INT2_PIN) == HIGH) {
      acc.readAccelMagnitude();
    }
    dataReady = false;
  }

  if (!recording) ledSet = false;

  //main data collection only when data ready isr sets the flag
  if (dataReady) {
    dataReady = false;  //clear flag so it can be set by interrupt on next data event
    //use isr timestamp not current time
    uint32_t sampleTs = isrTimestamp;
    //read acceleration and capacitance if available
    int32_t mag = acc.readAccelMagnitude();
    //debug:
    // Serial.println(mag);
    if (cdc.dataReady()) {
      zohCapacitance = cdc.readCapacitanceRaw();
    }

    if (recording) {
      uint32_t ts = sampleTs - start_time;
      FlashPacket pkt;
      if (ts < 100 || ts > 3600000000UL) {
        //too cose to start, start_time may not have settled
        //skip this packet
      } else {
        pkt.timestamp = ts;
        //check if the packets are right before they are written
        // Serial.print("timestamp: ");
        // Serial.println(pkt.timestamp);
        //we know for sure the time stamps are good
        pkt.mag_accel = mag;
        pkt.capacitance = zohCapacitance;
        pkt.reserved = 0;

        if (!flashLogger.appendPacket(pkt)) {
          Serial.println("WARNING: flash buffer overflow — stopping recording");
          recording = false;
          leds.End_Recording();
        }
      }
    }
  } else {
    //when there's no data it's our chance to service flash
    //so we don't block data ready event
    flashLogger.serviceFlashWrite();
  }
}
