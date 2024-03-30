#pragma once

#include "MovingAverageFilter.hpp"
#include "UltrasonicSensor.hpp"

namespace mtrn3100 {

template <size_t N>
class MAFUltrasonicSensor : public UltrasonicSensor {
public:
    MAFUltrasonicSensor(UltrasonicSensor& sensor) : UltrasonicSensor(sensor) {}

    // The sensor is ready to be used if the filter is full.
    bool isReady() const { return filter.isFull(); }

    // Take a sample from the sensor and put it into the filter.
    void sample() {
        float dist = echo();
        filter.sample(dist);
    }

    // Returns the average of the filter.
    float value() const { return filter.average(); }

    // Returns the capacity of the filter.
    size_t capacity() const { return filter.capacity(); }

private:
    MovingAverageFilter<float, N> filter;
};

}  // namespace mtrn3100