//This is a test file using simple interrupts and super loop

#include "flash/flash.hpp"
#include "accel/Accelerometer.hpp"
#include "cdc/CDCConverter.hpp"
#include "leds/LEDs.hpp"

#include <Wire.h>

//objects

TwoWire I2C_CDC = TwoWire(0);   // SDA=11, SCL=10
TwoWire I2C_ACC = TwoWire(1);   // SDA=41, SCL=40

// ── Peripheral objects ────────────────────────────────────────────────────────
LEDs leds;
CDC  cdc(I2C_CDC);
ACC  acc(I2C_ACC);

// ── Flash logger ──────────────────────────────────────────────────────────────
// 64 packets per buffer — at 800 Hz that gives 80 ms of headroom per buffer,
// which comfortably covers the worst-case SPI sector-erase time (~50 ms).
flash<64> flashLogger;
#define LOG_BASE_ADDR  0x000000
#define LOG_SIZE_BYTES 0x1000000  // 16 MB — full 128 Mbit chip

// --ISRs _________________________________
volatile bool recording =false;
volatile bool dataReady =false;
volatile uint32_t start_time =0;
uint32_t zohCapacitance =0;

void IRAM_ATTR onActivityISR(){
  if(!recording){
    recording = true;
    start_time = micros();
  }
}

void IRAM_ATTR onDataReadyISR(){
  dataReady = true; //just set the flag so the loop gets the newest value
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
        leds.Read_Data();
        Serial.println("Erasing flash...");
        flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
        flashLogger.eraseLogRegion();
        Serial.println("Flash erased.");
        leds.End_Read_Data();
    }
}

//setup
void setup(){
  leds.Init();
  Serial.begin(115200);
  I2C_CDC.begin(11,10,400000);
  I2C_ACC.begin(41,40,400000);
  if (!cdc.Setup())  Serial.println("ERROR: CDC not found");
  else               Serial.println("CDC ok");

  if (!acc.Setup())  Serial.println("ERROR: ACC not found");
  else               Serial.println("ACC ok");

  if (!flashLogger.begin()) {
      Serial.println("ERROR: Flash not found");
      while (true) { leds.ThresholdA_Reached(); delay(500); }
  }
  flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);

  // Clear INT2 before attaching — safe here, nothing else running

  attachInterrupt(digitalPinToInterrupt(ACC_INT1_PIN), onActivityISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ACC_INT2_PIN), onDataReadyISR, RISING);

  Serial.println("Smart Pick ready. Waiting for activity to begin recording...");
}

//loop---
void loop(){
  if (Serial.available()) {
    checkSerialCommand();
    return;
    }

  static bool ledSet = false;
  if(recording && !ledSet){
    ledSet = true;
    leds.Start_Recording();
    Serial.println("Activity detected - recording started");
    //make sure data ready interrupt pin goes low 
    while (digitalRead(ACC_INT2_PIN) == HIGH) {
      acc.readAccelMagnitude();
    }
  }

  if (!recording) ledSet = false;

  //main data collection only when data ready isr sets the flag
  if(dataReady){
    dataReady = false; //clear flag 
    //read acceleration and capacitance if available
    uint32_t mag = acc.readAccelMagnitude();
    if(cdc.dataReady()){
      zohCapacitance = cdc.readCapacitanceRaw();
    }

    if (recording){
      FlashPacket pkt;
      pkt.timestamp = micros()-start_time;
      pkt.mag_accel = mag;
      pkt.capacitance =zohCapacitance;

      if (!flashLogger.appendPacket(pkt)) {
        Serial.println("WARNING: flash buffer overflow — stopping recording");
        recording = false;
        leds.End_Recording();
      }
    }
  }
}

