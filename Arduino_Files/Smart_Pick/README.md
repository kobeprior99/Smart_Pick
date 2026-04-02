# Smart Pick – Embedded Data Acquisition System

## Overview
This project is part of **EDNS 492 – Senior Design** and implements a high-speed embedded data acquisition system for rock mining applications. The system captures:

- Acceleration data (impact detection, and vibration sensing)
- Capacitance data (material sensing)

Data is timestamped, buffered, and stored in external SPI flash memory for later extraction and analysis.

---

## ⚠️ Documentation Note
> **Generative AI was used and carefully verified for documentation of code, including README files and file headers.**

---

## System Architecture

### Core Components
- **Accelerometer (ADXL375)** – detects impacts and triggers recording
- **Capacitance-to-Digital Converter (AD7746)** – measures material interaction
- **External SPI Flash (16 MB)** – stores logged data
- **LED Indicators** – provide system state feedback
- **ESP32-S3** –  microcontroller

---

## File Structure
/flash
flash.hpp → Flash logging system (double-buffered)

/accel
Accelerometer.hpp → ADXL375 interface (I2C)

/cdc
CDCConverter.hpp → AD7746 interface (I2C)

/leds
LEDs.hpp → Status LED control


---

## How the System Works

1. **Idle State**
   - System waits for accelerometer activity interrupt

2. **Recording Trigger**
   - Activity ISR starts recording
   - Timestamp initialized

3. **Data Collection**
   - Accelerometer interrupt (800 Hz) triggers sampling
   - Capacitance sampled when ready
   - Data stored in RAM buffers

4. **Flash Logging**
   - Double-buffer (Red/Blue) system prevents data loss
   - Flash writes occur during idle time between samples

5. **Data Extraction**
   - Data is sent over serial in CSV format
   - Python script captures and saves data

---

## Uploading Code to the Custom PCB

### Steps

1. **Enter Boot Mode**
   - Hold **BOOT**
   - Tap **ENABLE**
   - Release **BOOT**

2. **Arduino IDE Setup**

Configure the board as follows:

- **Board**: ESP32-S3 Dev Module  
- **USB CDC On Boot**: Enabled  
- **CPU Frequency**: 240 MHz  
- **Core Usage**: Events run on Core 1  
- **Flash Mode**: QIO 80 MHz  
- **Flash Size**: 16 MB (128 Mb)  
- **Partition Scheme**:  
  `16M Flash (2MB APP / 12.5MB FATFS)`  
- **Upload Mode**: USB-OTG CDC (TinyUSB)  
- **Upload Speed**: 921600  
- **USB Mode**: Hardware CDC and JTAG  

3. **Upload**
   - Select correct COM port
   - Click **Upload**

---

## Hardware Setup

### Connections

- **Battery**
  - Connect via onboard **JST connector**

- **Load Cell / Sensor**
  - Connect via designated **JST connector**

Ensure all connections are secure before powering the system.

---

## LED Indicators

| LED | Meaning |
|-----|--------|
| Red | Recording active (BLUE LED in actual)|
| B1  | Data read operation |
| B2  | Threshold A triggered |
| B3  | Threshold B triggered |

---

## Serial Commands

| Command | Function |
|--------|---------|
| `R` | Read all logged data (CSV output) |
| `E` | End recording and flush data |
| `D` | Erase flash memory |
| `T` | Run flash integrity test |
| `I` | I followed by an integer positive or negative increments CAPDAC |

---

## Data Extraction (Python)

1. Navigate to: PythonFile
run python3 main.py and follow graphical user interface instructions

## Data Format 
timestamp_us, acceleration, capacitance_raw(24bits)


