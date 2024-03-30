#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t trigger_pin_val, uint8_t echo_pin_val) : trigger_pin{trigger_pin_val}, echo_pin{echo_pin_val} {
        pinMode(trigger_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
    }

    float echo() const {
        // Clears the trigPin
        digitalWrite(trigger_pin, LOW);
        delay(2);
        
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigger_pin, HIGH); 
        delay(10);
        digitalWrite(trigger_pin, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        float duration = pulseIn(echo_pin, HIGH);
        
        // Conversion to distance
        return (v * duration) / 2000;
    }

private:
    const uint8_t trigger_pin;
    const uint8_t echo_pin;
    const int v = 343;
};

}  // namespace mtrn3100