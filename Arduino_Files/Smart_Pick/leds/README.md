 ## LEDs
 | LED  | GPIO Pin |
 |------|----------|
 | Red  | IO2      |
 | B1   | IO43     |
 | B2   | IO44     |
 | B3   | IO42     |
Init: cylce through the LEDS on Reset to confirm functioning:

Red LED: Turn on during recording (after Accelerometer interrupt)

B1 LED: Turn on during read, when the serial available is ready and the user types "R"

B2 LED: Flash if threshold A force is captured at load cell

B3 LED: Flash if threshold B force is captured at load cell 
