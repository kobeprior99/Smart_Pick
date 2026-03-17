//This is a test file using simple interrupts and super loop


// --ISRs _________________________________
volatile bool recording =false;
volatile bool dataReady =false;
volatile uint32_t start_time =0;
uint32_t zohCapacitance =0;

void IRAM_ATTR onActivityISR(){
  if(!recording){
    recording = true;
    startTime = micros();
  }
}

void IRAM_ATTR onDataReadyISR(){
  dataReady = true; //just set the flag so the loop gets the newest value
}


//serial handler
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
    Serial.pritnln("Activity detected - recording started");
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
      pkt.timestamp = micros()-startTime;
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

