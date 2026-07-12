#pragma once

#include "common.hpp"

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


namespace HAL {
namespace Sleep {

inline void goToSleep(uint8_t mode) {
    cli();
    // avr-libc's set_sleep_mode macro internally negates a mask,
    // tripping -Wsign-conversion at every expansion site
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
    set_sleep_mode(mode);
#pragma GCC diagnostic pop
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}


}
}
