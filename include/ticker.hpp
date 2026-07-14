#pragma once

#include "common.hpp"

#include <stdint.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h> 

namespace HAL {
namespace Ticker {

void setupMSTimer();
uint32_t getNumTicks();
uint32_t getMicros();
void pause();
void resume(uint16_t);

}
}
