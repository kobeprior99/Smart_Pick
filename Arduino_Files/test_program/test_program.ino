// GPIO pin definitions
const int LED_RED = 2;
const int LED_B1  = 44;
const int LED_B2  = 43;
const int LED_B3  = 42;

int leds[] = {LED_RED, LED_B1, LED_B2, LED_B3};
const int numLEDs = 4;

const int delayTime = 300; // milliseconds

void setup() {
  // Configure pins as outputs
  for (int i = 0; i < numLEDs; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < numLEDs; i++) {

    // Turn all LEDs off
    for (int j = 0; j < numLEDs; j++) {
      digitalWrite(leds[j], LOW);
    }

    // Turn current LED on
    digitalWrite(leds[i], HIGH);

    delay(delayTime);
  }
}
