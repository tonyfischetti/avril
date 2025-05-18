#pragma once

#include "common.hpp"

#include <stdint.h>

/**
 * An implementation of an 8-bit fibonacci linear feedback shift register
 * using a maximally long period (taps: 00011101)
 *
 * Calls out to hand-written assembly
 */

namespace HAL {
namespace Utils {
namespace Random {

struct LFSR {
    static void init(uint8_t seed);
    static uint8_t nextByte();
};

}
}
}
