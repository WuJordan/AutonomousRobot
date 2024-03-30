#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp = 1, float ki = 1, float kd = 1, float deadband = 1, float upperLim = 255, float lowerLim = 0)
        : kp(kp), ki(ki), kd(kd), deadband(deadband), upperLim(upperLim), lowerLim(lowerLim) {}

    float compute(float input) {
        const uint32_t curr_time = micros();
        const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - encoderOffset);
        
        // Serial.print("Erro: ");
        // Serial.println(error);


        if (fabs(error) < deadband) {
            prev_error = 0;
            return 0;
        }

        integral = prev_integral + (error * dt);    // COMPLETE THIS LINE.
        derivative = (error - prev_error) / dt;  // COMPLETE THIS LINE.
        float signal = (ki * integral) + (kd * derivative) + (kp * error);

        output = limit(signal, lowerLim, upperLim); 

        prev_integral = integral;
        prev_error = error;

        return output;
    }

    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void setDeadband(float limit) { deadband = limit; }

    void zeroEncoderAndSetTarget(float currentEncoderCount, float setpointInput) {
        encoderOffset = currentEncoderCount;
        setpoint = setpointInput; // setpoint = target from pid_position_control.ino
        prev_error = 0;
        prev_integral = 0;
    }

    static float limit(float val, float lower, float upper) {
      bool sign = val < 0;
      float absval = fabs(val);
      if (absval < lower) return sign ? -lower : lower;
      if (absval > upper) return sign ? -upper : upper;
      return val;
    }

private:
    float kp;
    float ki;
    float kd;
    float lowerLim;
    float upperLim;
    float deadband;
    float error;
    float derivative;
    float integral;
    float output;
    float prev_integral = 0;
    float prev_error = 0;
    float setpoint = 0;
    float encoderOffset = 0;
    uint32_t prev_time = micros();
};

}  // namespace mtrn3100