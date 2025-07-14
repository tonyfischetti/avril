#pragma once

#include "common.hpp"

#include <stdint.h>
#include "ticker.hpp"


namespace HAL {

namespace Utils {

template<typename T=uint16_t>
class Stopwatch {

    uint32_t started  { 0 };
    T countdownFrom   { 0 };

  public:

    Stopwatch()
        : started       { 0 },
          countdownFrom { 0 } {
    }

    void start() {
        started = HAL::Ticker::getNumTicks();
    }

    uint32_t elapsedTime() {
        return HAL::Ticker::getNumTicks();
    }

    void setExpiry(T expiry) {
        countdownFrom = expiry;
    }

    bool expired() {
        return ((HAL::Ticker::getNumTicks() - started) >= countdownFrom);
    }

};

}
}
