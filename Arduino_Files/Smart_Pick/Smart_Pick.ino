/* ================================================================
   Title:        Smart Pick For Rock Mining
   Date:         February 15, 2026
   Authors:      Kobe Prior, Sophia Mimlitz

   Brief:
   This code collects load cell data via capacitance change and
   acceleration data from an onboard accelerometer. The collected
   data is formatted and logged to flash memory

   Architecture:
   - Core 1: ACC data-ready ISR → accQueue → packerTask
             cdcPollerTask updates ZOH capacitance value
   - Core 0: loop() handles serial commands
             flashWriterTask drains ping-pong buffers to SPI flash

   Interrupt mapping (ADXL375):
   - INT1 (GPIO 36): single-shock → triggers recording start
   - INT2 (GPIO 35): data-ready  → fires at 800 Hz, wakes packerTask

   Zero-order hold:
   - CDC samples at ~90 Hz. Between CDC samples the last known
     capacitance value is reused so every 800 Hz ACC packet gets
     a valid capacitance field with no gaps.
   ================================================================
*/

#include "flash/flash.hpp"
#include "accel/Accelerometer.hpp"
#include "cdc/CDCConverter.hpp"
#include "leds/LEDs.hpp"

#include <Wire.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ── I2C buses ────────────────────────────────────────────────────────────────
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

// ── Shared state ──────────────────────────────────────────────────────────────

// Set true by the shock ISR, read by packerTask and loop().
// volatile ensures the compiler never caches this in a register.
static volatile bool recording = false;
static volatile bool recording_for_led = false;

// Timestamp of the shock event, captured in the ISR.
// Used to compute relative timestamps in every packet.
static volatile uint32_t startTime = 0;

// Zero-order hold: last valid capacitance reading from the CDC.
// Written by cdcPollerTask, read by packerTask — must be atomic so
// a half-written value is never observed on the other core.
static std::atomic<uint32_t> zohCapacitance = 0;

// Queue between the ACC data-ready ISR and packerTask.
// The ISR pushes a single token (the raw micros() timestamp at which
// the data-ready pin fired). packerTask blocks on this queue — it sleeps
// at zero CPU cost until a token arrives, then does the I2C read.
// Depth 128: at 800 Hz this is 160 ms of headroom before any token is lost.
static QueueHandle_t accQueue = nullptr;

// Mutex protecting the flashLogger object.
// Both packerTask (appendPacket) and flashWriterTask (serviceFlashWrite)
// touch the flash logger — the mutex ensures they never overlap.
static SemaphoreHandle_t flashMutex = nullptr;

// ── ISR handlers (must live in IRAM so they run even during flash ops) ────────

/*
 * onShockISR — fires on GPIO 36 rising edge (INT1, single-shock).
 *
 * We cannot do I2C here (I2C driver uses its own interrupts internally and
 * nesting them causes a crash on the ESP32). So we just set the recording
 * flag and capture the start time. The INT_SOURCE register that clears the
 * ADXL375 interrupt is read from packerTask on its first iteration instead.
 */
void IRAM_ATTR onShockISR() {
    if (!recording) {
        recording  = true;
        recording_for_led = true;
        startTime  = micros();
    }
}

/*
 * onDataReadyISR — fires on GPIO 35 rising edge (INT2, data-ready, 800 Hz).
 *
 * We push the current microsecond timestamp onto accQueue so packerTask
 * knows exactly when this sample was due, even if it wakes up a fraction
 * late. portYIELD_FROM_ISR() asks the scheduler to immediately switch to
 * packerTask if it was unblocked by this send (avoids an extra tick delay).
 */
void IRAM_ATTR onDataReadyISR() {
    //always push -packerTask filters on recording flag
    //this ensures we never miss the first rising edge after recording starts
    uint32_t ts = micros();
    BaseType_t higherPriorityWoken = pdFALSE;
    xQueueSendFromISR(accQueue, &ts, &higherPriorityWoken);
    portYIELD_FROM_ISR(higherPriorityWoken);
}

// ── Task: CDC poller (Core 1, priority 4) ────────────────────────────────────
/*
 * Wakes every 11 ms (~90 Hz) using vTaskDelayUntil, which is drift-free —
 * it accounts for how long the task body took so the period stays accurate.
 *
 * If the CDC has a new sample, atomically store it as the ZOH value.
 * If not, do nothing — the old value stays, which is the zero-order hold.
 */
void cdcPollerTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(11);  // 11 ms ≈ 90 Hz

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        if (!recording) continue;

        if (cdc.dataReady()) {
            uint32_t raw = cdc.readCapacitanceRaw();
            zohCapacitance.store(raw);
        }
        // No new CDC data → ZOH holds last value automatically
    }
}

// ── Task: packer (Core 1, priority 5 — highest on this core) ─────────────────
/*
 * Blocks indefinitely on accQueue. Every time the data-ready ISR fires and
 * pushes a timestamp token, this task unblocks, reads the accelerometer over
 * I2C (safe here — we are in a task, not an ISR), snapshots the ZOH
 * capacitance, builds a FlashPacket, and appends it to the flash logger.
 *
 * First iteration after shock: also clears the ADXL375 INT_SOURCE register
 * that the ISR could not safely read.
 */
void packerTask(void* pvParameters) {
    uint32_t sampleTs = 0;

    for (;;) {
        if (xQueueReceive(accQueue, &sampleTs, portMAX_DELAY) != pdTRUE) continue;
        //You have to read no matter what, these will be lost but it's necessary to clear interrupt


        //only log if we are actually recording:
        if (!recording){
            acc.readAccelMagnitude();

            continue;
        } //filter here instead of isr
        uint32_t mag = acc.readAccelMagnitude();
        uint32_t cap = zohCapacitance.load();
        FlashPacket pkt;
        pkt.timestamp   = sampleTs - startTime;
        pkt.mag_accel   = mag;
        pkt.capacitance = cap;

        if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool ok = flashLogger.appendPacket(pkt);
            xSemaphoreGive(flashMutex);

            if (!ok) {
                Serial.println("WARNING: flash buffer overflow — stopping recording");
                recording = false;
                leds.End_Recording();
            }
        }
    }
}

// ── Task: flash writer (Core 0, priority 3) ───────────────────────────────────
/*
 * Polls serviceFlashWrite() every 5 ms. When a full buffer is ready the call
 * does a page-program SPI burst and returns; otherwise it returns immediately.
 * Running on Core 0 means SPI flash writes never compete with the ACC ISR or
 * the packer/CDC tasks on Core 1.
 */
void flashWriterTask(void* pvParameters) {
    for (;;) {
        if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            flashLogger.serviceFlashWrite();
            xSemaphoreGive(flashMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ── Serial command handler (runs in loop() on Core 0) ────────────────────────
/*
 * Commands (send via Serial Monitor, line ending = newline):
 *   R — dump all logged packets as CSV over serial
 *   E — end recording, finalize (flush partial buffer to flash)
 *   D — erase entire flash log region
 */
void checkSerialCommand() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    // 'R' — read all logged packets back over serial as CSV
    if (cmd == 'R' || cmd == 'r') {
        leds.Read_Data();

        // Pause recording while we read so packerTask doesn't keep appending
        bool wasRecording = recording;
        recording = false;

        // Finalize any partial buffer first
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
            flashLogger.finalizeLog();
            xSemaphoreGive(flashMutex);
        }

        uint32_t totalBytes = flashLogger.getPacketsLogged() * flashLogger.getPacketSize();
        Serial.println("timestamp_us,acceleration,capacitance");

        FlashPacket fp;
        // Serial.print("Packet size: ");//prints 12 (bytes)as expected
        // Serial.println(flashLogger.getPacketSize());
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
        //helpful for python file to know when to stop
        Serial.println("EOF");

        recording = wasRecording;
        leds.End_Read_Data();
    }

    // 'E' — end recording, flush to flash
    if (cmd == 'E' || cmd == 'e') {
        recording = false;
        leds.End_Recording();

        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
            flashLogger.finalizeLog();
            xSemaphoreGive(flashMutex);
        }

        Serial.print("Recording ended. Packets logged: ");
        Serial.println(flashLogger.getPacketsLogged());
    }

    // 'D' — erase flash
    if (cmd == 'D' || cmd == 'd') {
        leds.Read_Data();
        Serial.println("Erasing flash...");

        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
            flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);
            flashLogger.eraseLogRegion();
            xSemaphoreGive(flashMutex);
        }

        Serial.println("Flash erased.");
        leds.End_Read_Data();
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    // LEDs first — startup flash sequence confirms power and pin config
    leds.Init();

    // USB-CDC serial (Tools > USB CDC On Boot must be enabled)
    Serial.begin(115200);

    // I2C buses
    I2C_CDC.begin(11, 10, 400000);   // SDA=11, SCL=10, 400 kHz
    I2C_ACC.begin(41, 40, 400000);   // SDA=41, SCL=40, 400 kHz

    // Peripheral setup
    if(!cdc.Setup()){
        Serial.println("Error: cdc not found check i2c wiring");
    }else{
        Serial.println("cdc is chillin");
    }

    if(!acc.Setup()){
      Serial.println("Error: adxl375 not found check i2c wiring");
    }else{
      Serial.println("adxl375 ok");
    }
    //acc.debugPrint();
    

    // Flash setup — begin() initialises SPI and verifies the JEDEC ID
    if (!flashLogger.begin()) {
        Serial.println("ERROR: Flash not found — check SPI wiring");
        while (true) { leds.ThresholdA_Reached(); delay(500); }
    }
    flashLogger.configureLogRegion(LOG_BASE_ADDR, LOG_SIZE_BYTES);

    // FreeRTOS primitives — create before attaching interrupts
    accQueue   = xQueueCreate(128, sizeof(uint32_t));
    flashMutex = xSemaphoreCreateMutex();

    if (!accQueue || !flashMutex) {
        Serial.println("ERROR: FreeRTOS object creation failed");
        while (true) { leds.ThresholdA_Reached(); delay(500); }
    } 
    acc.readAccelMagnitude();
    // Hardware interrupts — attach after queue exists so ISR never fires
    // into a null handle
    // Serial.print("INT2 before attachInterrupt: ");
    // Serial.println(digitalRead(ACC_INT2_PIN));

    attachInterrupt(digitalPinToInterrupt(ACC_INT1_PIN), onShockISR,    RISING);
    attachInterrupt(digitalPinToInterrupt(ACC_INT2_PIN), onDataReadyISR, RISING);

    // Serial.print("INT2 before attachInterrupt: ");
    // Serial.println(digitalRead(ACC_INT2_PIN));
    // ── FreeRTOS tasks ──────────────────────────────────────────────────────
    // xTaskCreatePinnedToCore(function, name, stack bytes, param, priority, handle, core)
    //
    // Core 1 — time-critical sensing
    xTaskCreatePinnedToCore(packerTask,     "packer",  4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(cdcPollerTask,  "cdc",     2048, NULL, 4, NULL, 1);
    // Core 0 — flash writes and serial (shares core with loop())
    xTaskCreatePinnedToCore(flashWriterTask,"flash",   2048, NULL, 3, NULL, 0);

    Serial.println("Smart Pick ready. Waiting for shock to begin recording...");
}

// ── loop (Core 0, priority 1) ─────────────────────────────────────────────────
/*
 * loop() stays on Core 0 and only handles serial commands.
 * vTaskDelay yields CPU to flashWriterTask between serial checks —
 * without it loop() would starve the flash writer.
 */
void loop() {
    checkSerialCommand();
    if (recording_for_led) {
        leds.Start_Recording();
        Serial.println("Shock detected — recording started.");
        recording_for_led = false;
        //go ahead and perform read to clear data ready interrupt
        while (digitalRead(ACC_INT2_PIN) == HIGH){
            acc.readAccelMagnitude();//read until the interrupt gets cleared
        }
    }
    //Temporary: print live accel to find shock threshold
    // static uint32_t lastPrint = 0;
    // if (millis() - lastPrint > 200) {
    //     acc.debugAccel();
    //     lastPrint = millis();
    // }
        //'Temporary diagnostics
    // static uint32_t lastPrint = 0;
    // if (millis() - lastPrint > 500) {
    //     Serial.print("recording: ");     Serial.println(recording);
    //     Serial.print("queue depth: ");   Serial.println(uxQueueMessagesWaiting(accQueue));
    //     Serial.print("packets logged: ");Serial.println(flashLogger.getPacketsLogged());
    //     Serial.print("INT2 pin: ");      Serial.println(digitalRead(ACC_INT2_PIN));
    //     lastPrint = millis();
    // }
    vTaskDelay(pdMS_TO_TICKS(10));
}
