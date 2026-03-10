#ifndef LEDS_HPP
#define LEDS_HPP
#include <Arduino.h>

class LEDs {
private:
    static const int RED_PIN = 42;
    static const int B1_PIN  = 44;
    static const int B2_PIN  = 43;
    static const int B3_PIN  = 2;

    void flash(int pin, int duration = 150) {
        digitalWrite(pin, HIGH);
        delay(duration);
        digitalWrite(pin, LOW);
        delay(duration);
    }

public:
    void init() {
        // Configure pins as outputs
        pinMode(RED_PIN, OUTPUT);
        pinMode(B1_PIN, OUTPUT);
        pinMode(B2_PIN, OUTPUT);
        pinMode(B3_PIN, OUTPUT);

        // Ensure all LEDs start OFF
        digitalWrite(RED_PIN, LOW);
        digitalWrite(B1_PIN, LOW);
        digitalWrite(B2_PIN, LOW);
        digitalWrite(B3_PIN, LOW);

        // Startup flash sequence
        flash(RED_PIN);
        flash(B1_PIN);
        flash(B2_PIN);
        flash(B3_PIN);
    }
};

#endif
