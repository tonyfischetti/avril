#pragma once

#include "common.hpp"

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


namespace HAL {
namespace Sleep {

inline void goToSleep(uint8_t mode) {
    cli();
    set_sleep_mode(mode);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}


}
}
