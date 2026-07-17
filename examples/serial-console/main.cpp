/**
 * serial-console: single-key commands from your terminal -- the
 * simplest real use of UART receive.
 *
 * Open a serial terminal at 9600 baud and press keys:
 *
 *   1   LED on              v   supply voltage (mV)
 *   0   LED off             t   on-die temperature (raw)
 *   b   toggle blink mode   ?   this menu
 *
 * Anything else echoes back inside [brackets] so you can see exactly
 * what arrived. Receive errors are REPORTED, not swallowed -- set
 * your terminal to the wrong baud rate on purpose and watch
 * "[frame error -- baud mismatch?]" diagnose it in real time. That's
 * tryRead()'s tier-2 error honesty doing its job.
 *
 * Why single-key instead of typed lines: polled RX has no buffer
 * beyond the hardware's 2-deep FIFO, so the loop must visit
 * tryRead() faster than characters arrive. At 9600 baud a character
 * lasts ~1.04 ms and the FIFO grants ~3 character times (~3 ms) of
 * grace; this loop wakes every millisecond (IDLE + ticker), so even
 * paste-a-paragraph won't drop bytes -- but a line editor (echo,
 * backspace, a buffer, a parser) would triple the example's size for
 * no extra lesson. When something machine-fast talks to the 328P,
 * that's the (future) interrupt-driven ring buffer's job, not
 * polling's.
 *
 * Parts: ATmega328P @ 16 MHz, a USB-serial adapter, one LED + ~330R.
 *
 * Wiring (physical DIP-28 pins):
 *   UART RX -> PD0 (2)   from the adapter's TX (cross over!)
 *   UART TX -> PD1 (3)   to the adapter's RX
 *   LED     -> PB0 (14), through ~330R to GND
 *
 * Exercises: UART (both directions at last -- tryRead's RxStatus,
 * println/print_P), Analog (the free meters), Ticker, Sleep.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"

namespace UART = HAL::Comms::UART;

using Led = HAL::GPIO::GPIO<14>;   // PB0

bool blinkP { false };

void help() {
    UART::println_P(PSTR("serial-console:"));
    UART::println_P(PSTR("  1  LED on       v  Vcc (mV)"));
    UART::println_P(PSTR("  0  LED off      t  temp (raw)"));
    UART::println_P(PSTR("  b  blink mode   ?  this menu"));
}

void handle(uint8_t c) {
    switch (c) {
        case '1':
            blinkP = false;
            Led::setHigh();
            UART::println_P(PSTR("LED on"));
            break;
        case '0':
            blinkP = false;
            Led::setLow();
            UART::println_P(PSTR("LED off"));
            break;
        case 'b':
            blinkP = !blinkP;
            UART::println_P(blinkP ? PSTR("blinking")
                                   : PSTR("blink off"));
            if (!blinkP) Led::setLow();
            break;
        case 'v':
            UART::print_P(PSTR("vcc: "));
            UART::print(HAL::Analog::readVccMillivolts());
            UART::println_P(PSTR(" mV"));
            break;
        case 't':
            UART::print_P(PSTR("temp raw: "));
            UART::println(HAL::Analog::readTemperature());
            break;
        case '?':
        case 'h':
            help();
            break;
        case '\r':
        case '\n':
            break;   // terminals send these; silence, not sass
        default:
            // echo the mystery byte so you can SEE what arrived
            UART::print_P(PSTR("["));
            UART::printByte(c);
            UART::print_P(PSTR("] ? for help"));
            UART::println("");
            break;
    }
}

int main() {
    HAL::Ticker::setupMSTimer();
    HAL::Analog::begin();
    UART::init<9600>();
    Led::setOutput();
    sei();

    UART::println_P(PSTR("serial-console ready (? for help)"));

    while (1) {
        // drain everything waiting (paste-friendly), reporting the
        // hardware's per-byte verdict instead of swallowing it
        uint8_t c;
        UART::RxStatus rs;
        while ((rs = UART::tryRead(c)) != UART::RxStatus::NONE) {
            switch (rs) {
                case UART::RxStatus::OK:
                    handle(c);
                    break;
                case UART::RxStatus::FRAME_ERROR:
                    UART::println_P(
                        PSTR("[frame error -- baud mismatch?]"));
                    break;
                case UART::RxStatus::OVERRUN:
                    // this byte is fine; EARLIER bytes were lost
                    UART::println_P(PSTR("[overrun: bytes lost]"));
                    handle(c);
                    break;
                default:   // PARITY_ERROR: impossible in 8N1; NONE:
                    break; // unreachable inside the loop
            }
        }

        if (blinkP) {
            uint32_t ticks { HAL::Ticker::getNumTicks() };
            if (ticks & 0x100) Led::setHigh(); else Led::setLow();
        }

        HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
    }
}
