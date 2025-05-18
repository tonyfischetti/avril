#pragma once

#include <stdint.h>
#include "util/LFSR.hpp"


extern "C" {
    void init_lfsr(uint8_t seed);
    uint8_t get_next_byte(void);
}

namespace HAL::Utils::Random {

void LFSR::init(uint8_t seed) {
    init_lfsr(seed);
}

uint8_t LFSR::nextByte() {
    return get_next_byte();
}

}
