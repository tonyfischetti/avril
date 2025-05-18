#pragma once

#include "common.hpp"

#include <stdio.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


namespace HAL {
namespace UART {

#if defined(__AVR_ATtiny85__)
#warning "HAL::UART is not supported on ATtiny85"
#else

template<uint32_t BaudRate>
inline void init() {
    static_assert(BaudRate > 0, "Baud rate must be > 0");

    constexpr uint16_t ubrr = (F_CPU / (16UL * BaudRate)) - 1;

    // Set baud rate
    UBRR0H = (uint8_t)(ubrr>>8);
    UBRR0L = (uint8_t)ubrr;
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

    // Enable transmitter
    //  TODO  parameterize
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

inline void printByte(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1<<UDRE0))) {}
    
    // Put data into buffer, sends the data
    UDR0 = data;
}

inline void print(const char* str) {
    while (*str) {
        printByte(*str++);
    }
}

inline void println(const char* str) {
    print(str);
    printByte('\n');
}

//  TODO  
inline void print(uint32_t n) {
    char buf[20];
    snprintf(buf, 20, "%lu", n);
    println(buf);
}

#endif

}
}
