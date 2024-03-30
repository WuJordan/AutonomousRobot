  #pragma once

  #include <Arduino.h>

  #include <math.h>

  namespace mtrn3100 {

  class Encoder {
  public:
      Encoder(uint8_t enc1, uint8_t enc2, void* callback) : encoder1_pin(enc1), encoder2_pin(enc2) {
          pinMode(encoder1_pin, INPUT_PULLUP);
          pinMode(encoder2_pin, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(encoder1_pin), callback, RISING);
      }

      void readEncoder() {
          noInterrupts();
          if (digitalRead(encoder2_pin)) {
              direction = 1;
          } else {
              direction = -1;
          }
          interrupts();

          // Get motor position.
          // COMPLETE THIS BLOCK.
          noInterrupts();
          const uint16_t counts_per_revolution = 266;
          float increment = 1 / counts_per_revolution;
          position += static_cast<float>(direction) * (1.0 / static_cast<float>(counts_per_revolution) * 2 * M_PI);
          // if (position >= 2*M_PI) {
          //   position -= 2 * M_PI;
          // }
          // else if (position < 0) {
          //   position = 2 * M_PI + position;
          // }
          Serial.println(position);
          interrupts();
      }

  public:
      const uint8_t encoder1_pin;
      const uint8_t encoder2_pin;
      int8_t direction;
      float position;
      uint32_t prev_time;
      bool read = false;
  };

  // Not inside Encoder because only free functions are interruptable.
  void setEncoder(mtrn3100::Encoder& encoder) { encoder.read = true; }

  }  // namespace mtrn3100
