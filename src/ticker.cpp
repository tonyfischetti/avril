
#include <stdint.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h> 

namespace HAL {
namespace Ticker {

volatile uint32_t ticks = 0;
volatile bool paused = false;

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

void pause() {
    paused = true;
    TCCR0B = 0;
}

void resume(uint16_t compTicks) {
    if (paused) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ticks += compTicks;
        }
        paused = false;
        setupMSTimer();
    }
}

}
}


#if defined(__AVR_ATtiny85__)
ISR(TIM0_COMPA_vect) {
#elif defined(__AVR_ATmega328P__)
ISR(TIMER0_COMPA_vect) {
#endif
    HAL::Ticker::ticks++;
}
