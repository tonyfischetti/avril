
#include <stdint.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h> 

namespace HAL {
namespace Ticker {

volatile uint32_t ticks { 0 };
volatile uint8_t paused { 0 }; // takes on TCCR0B if paused, 0 otherwise

constexpr struct PrescalerOption {
    uint16_t prescaler;
    uint8_t cs_bits;
} prescaler_options[] = {
    {8,    (1 << CS01)},
    {64,   (1 << CS01) | (1 << CS00)},
    {256,  (1 << CS02)},
    {1024, (1 << CS02) | (1 << CS00)}
};

constexpr PrescalerOption selectPrescaler() {
    for (auto& opt: prescaler_options) {
        uint32_t ocr = F_CPU / (opt.prescaler * 1000UL);
        if (ocr > 0 && ocr <= 256) return opt;
    }
    // highest is better if nothing fits
    return prescaler_options[3];
}

constexpr bool isExactOCR0A(uint32_t f_cpu, uint16_t prescaler) {
    return (f_cpu % (prescaler * 1000UL)) == 0;
}

constexpr int findValidPrescalerIndex() {
    for (uint8_t i = 0; i < sizeof(prescaler_options) / sizeof(prescaler_options[0]); ++i) {
        if (isExactOCR0A(F_CPU, prescaler_options[i].prescaler)) {
            uint32_t ocr = F_CPU / (prescaler_options[i].prescaler * 1000UL);
            if (ocr > 0 && ocr <= 256)
                return i;
        }
    }
    return -1;
}

/**
 * We are looking to achieve 1000 Hz
 *   OCR0A = (F_CPU / (prescaler * 1000)) - 1
 */

void setupMSTimer() {
    constexpr int prescaler_index = findValidPrescalerIndex();
    static_assert(prescaler_index >= 0, "No valid prescaler found for exact 1ms tick with F_CPU");
    constexpr auto opt = prescaler_options[prescaler_index];
    constexpr uint8_t ocr = static_cast<uint8_t>((F_CPU / (opt.prescaler * 1000UL)) - 1);

    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = opt.cs_bits;
    OCR0A = ocr;

#if defined(__AVR_ATtiny85__)
    TIMSK |= (1 << OCIE0A);
#elif defined(__AVR_ATmega328P__)
    TIMSK0 |= (1 << OCIE0A);
#endif

}

uint32_t getNumTicks() {
    uint32_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = ticks;
    }
    return value;
}

/**
 * Microseconds since boot: the millisecond tick count combined with the
 * live TCNT0 fraction. Resolution is prescaler/MHz microseconds per
 * count (8 us at 8 MHz / 64, 4 us at 16 MHz / 64).
 *
 * Wraps every ~71.6 minutes; consumers must compare timestamps with
 * unsigned subtraction. Only meaningful while the ticker is running --
 * a paused timer returns a frozen value.
 */
uint32_t getMicros() {
    constexpr int prescaler_index = findValidPrescalerIndex();
    static_assert(prescaler_index >= 0,
            "No valid prescaler found for exact 1ms tick with F_CPU");
    static_assert(F_CPU % 1000000UL == 0,
            "getMicros() needs an integer-MHz F_CPU");
    constexpr uint32_t mhz = F_CPU / 1000000UL;
    constexpr uint32_t usPerCount =
        prescaler_options[prescaler_index].prescaler / mhz;
    static_assert(usPerCount * mhz ==
            prescaler_options[prescaler_index].prescaler,
            "Timer0 count period is not a whole number of microseconds");

    uint32_t ms;
    uint8_t  count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms    = ticks;
        count = TCNT0;
        // a compare match may be pending that the tick ISR hasn't yet
        // credited to `ticks`: either it landed after ATOMIC_BLOCK's
        // cli, or we were called from an ISR (interrupts already off,
        // so the tick ISR can't run). Credit the missing millisecond
        // and re-read the now-post-wrap counter. A flag left pending
        // for more than one full period (interrupts off > 1 ms) is
        // indistinguishable from this and under-reports by the excess
#if defined(__AVR_ATtiny85__)
        if (TIFR & (1 << OCF0A)) {
#elif defined(__AVR_ATmega328P__)
        if (TIFR0 & (1 << OCF0A)) {
#endif
            ms++;
            count = TCNT0;
        }
    }
    return ms * 1000UL + static_cast<uint32_t>(count) * usPerCount;
}

void pause() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // only pause a running timer; a second pause() must not
        // overwrite the saved prescaler bits with 0
        if (TCCR0B) {
            paused = TCCR0B;
            TCCR0B = 0;
        }
    }
}

void resume(uint16_t compTicks) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (paused) {
            ticks += compTicks;
            TCCR0B = paused;
            paused = 0;
        }
    }
}

}
}


#if defined(__AVR_ATtiny85__)
ISR(TIM0_COMPA_vect) {
#elif defined(__AVR_ATmega328P__)
ISR(TIMER0_COMPA_vect) {
#endif
    HAL::Ticker::ticks = HAL::Ticker::ticks + 1;
}
