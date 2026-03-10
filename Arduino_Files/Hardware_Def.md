# Smart Pick Tool – IO Definitions

## Flash Memory (SPI Interface)
| Signal | GPIO Pin |
|--------|----------|
| SCK    | IO47     |
| MOSI   | IO48     |
| MISO   | IO13     |
| CS     | IO14     |

---

## CDC (I2C Interface)
| Signal | GPIO Pin |
|--------|----------|
| SCL    | IO10     |
| SDA    | IO11     |

---

## Accelerometer (I2C Interface)
| Signal      | GPIO Pin |
|-------------|----------|
| SDA         | IO41     |
| SCL         | IO40     |
| Interrupt 1 | IO36     |
| Interrupt 2 | IO35     |

---

## LEDs
| LED  | GPIO Pin |
|------|----------|
| Red  | IO42     |
| B1   | IO44     |
| B2   | IO43     |
| B3   | IO2      |

---

## Notes
- SPI bus is dedicated to Flash memory.
- CDC and Accelerometer use separate I2C buses.
- Accelerometer interrupts are configured on dedicated GPIO pins.
- All GPIO numbering follows ESP-style `IOxx` naming convention.
