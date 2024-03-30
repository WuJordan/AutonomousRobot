#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class Motor {
public:
    // COMPLETE THIS CONSTRUCTOR.
    Motor(uint8_t analog, uint8_t input1, uint8_t input2) : analog_pin{analog}, input1_pin{input1}, input2_pin{input2} {
        pinMode(analog_pin, OUTPUT);
        pinMode(input1_pin, OUTPUT);
        pinMode(input2_pin, OUTPUT);
    }

    // COMPLETE THIS FUNCTION.
    // This function sets the PWM of the motor and returns nothing. This function accepts a signed PWM signal with range
    // [-255, 255]. Any values outside this range are clamped to the limits.
    void setPWM(int16_t pwm) {
        if (pwm < 0) {
            if (pwm < -255) {
                pwm = -255;
            }
            // input1_pin = 1;
            digitalWrite(input1_pin, LOW);
            // input2_pin = 0;
            digitalWrite(input2_pin, HIGH);
            pwm = abs(pwm);
        }
        else if (pwm > 0) {
            if (pwm > 255) {
                pwm = 255;
            }
            // input1_pin = 0;
            digitalWrite(input1_pin, HIGH);
            // input2_pin = 1;
            digitalWrite(input2_pin, LOW);
        }
        else if (pwm == 0) {
            // input1_pin = 1;
            digitalWrite(input1_pin, HIGH);
            // input2_pin = 1;
            digitalWrite(input2_pin, HIGH);
        }
        analogWrite(analog_pin, pwm);
    }

private:
    const uint8_t analog_pin;
    const uint8_t input1_pin;
    const uint8_t input2_pin;
};

}  // namespace mtrn3100
