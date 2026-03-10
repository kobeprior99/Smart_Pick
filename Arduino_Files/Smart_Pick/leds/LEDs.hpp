#ifndef LEDS_HPP
#define LEDS_HPP
#include <Arduino.h>

class LEDs {
private:
  
// ## LEDs
// | LED  | GPIO Pin |
// |------|----------|
// | Red  | IO2      |
// | B1   | IO43     |
// | B2   | IO44     |
// | B3   | IO42     |

    static const int RED_PIN = 2;
    static const int B1_PIN  = 43;
    static const int B2_PIN  = 44;
    static const int B3_PIN  = 42;

    void flash(int pin, int duration = 150) {
        digitalWrite(pin, HIGH);
        delay(duration);
        digitalWrite(pin, LOW);
        delay(duration);
    }

public:
    void Init() {
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
        delay(1000);
        // Startup flash sequence
        flash(RED_PIN);
        flash(B1_PIN);
        flash(B2_PIN);
        flash(B3_PIN);
    }
    void Start_Recording(){
     digitalWrite(RED_PIN, HIGH); 
    }

    void End_Recording(){
     digitalWrite(RED_PIN, LOW); 
    }

    void Read_Data(){
      digitalWrite(B1_PIN, HIGH);
    }

    void End_Read_Data(){
      digitalWrite(B1_PIN, LOW);
    }

    void ThresholdA_Reached(){
      flash(B2_PIN);
    }  

    void ThresholdB_Reached(){
      flash(B3_PIN);
    }  
};
  

#endif
