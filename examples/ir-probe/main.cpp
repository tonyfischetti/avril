/**
 * ir-probe: point any NEC remote at it and learn its codes.
 *
 * The tool you need BEFORE writing any other IR appliance: every
 * button press shows its address and command in hex on the LCD, with
 * a live repeat counter while the button is held -- and the same
 * stream goes out the UART as copy-pasteable lines, so building a
 * command table for a remote is: mash every button, save the log.
 *
 *   LCD row 0:   A:00 C:45
 *   LCD row 1:   press 003 rpt 027
 *   UART:        addr 00 cmd 45 .....................
 *                addr 00 cmd 46 ...
 *
 * (each '.' is one ~110 ms repeat frame -- hold a button and watch
 * the NEC repeat cadence draw itself)
 *
 * Non-NEC remotes produce nothing at all (the decoder discards
 * out-of-window timing), which is itself a diagnosis: silence means
 * "not NEC", not "broken".
 *
 * Parts: ATmega328P @ 16 MHz, TSOP4838 (or TSOP38238 -- same job,
 * DIFFERENT pinout, check the datasheet), LCD1602 + PCF8574
 * backpack, any NEC remote (the 21-key "car MP3" remotes are the
 * classic, and most TV/media remotes are NEC too).
 *
 * The TSOP wants Vishay's supply filter (100R series + >=0.1 uF to
 * GND at its Vs pin): the can's ~56x AGC amplifier is sensitive
 * enough that supply ripple reads as phantom IR. Breakout boards
 * usually include it; a bare can does not. Details: TSOP4838.md §1.
 *
 * Wiring (physical DIP-28 pins):
 *   TSOP OUT -> PD2 (4)      (any PCINT pin works; it idles HIGH on
 *                             its own internal pull-up)
 *   SDA -> PC4 (27), SCL -> PC5 (28)    LCD backpack
 *   UART TX -> PD1 (3), 9600 baud
 *
 * This is a bench tool: it stays awake in IDLE (getMicros() must
 * keep running to timestamp edges; there is no battery to protect).
 * Deliberately ABSENT: the driver's busy() sleep-gate predicate --
 * that guards PWR_DOWN (which freezes the us clock mid-frame), a
 * hazard IDLE doesn't have, since Timer0 keeps counting through it.
 * Any PWR_DOWN consumer must gate on !ir.busy(); see the driver
 * docs and TSOP4838.md §9.
 *
 * Exercises: IRReceiverNEC (the timestamp-first ISR discipline),
 * LCD1602, UART, Ticker, Sleep.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"
#include "devices/IRReceiverNEC.hpp"
#include "devices/LCD1602.hpp"

namespace UART = HAL::Comms::UART;

using I2c = HAL::Comms::I2C::Master<100000>;
using Lcd = HAL::Devices::LCD1602<I2c>;

HAL::Devices::IRReceiverNEC<4> ir;   // TSOP on PD2

volatile uint8_t previousPIND { 0xFF };

ISR(PCINT2_vect) {
    // the IR timestamp comes FIRST, before anything else the ISR
    // does -- other work must never become timestamp jitter
    uint32_t nowUs { HAL::Ticker::getMicros() };
    uint8_t  cur   { PIND };
    uint8_t  ch    { static_cast<uint8_t>(cur ^ previousPIND) };
    previousPIND = cur;
    ir.notifyInterruptOccurred(nowUs, HAL::GPIO::Port::D, ch);
}

uint16_t pressCount  { 0 };
uint16_t repeatCount { 0 };

void print3(uint16_t v) {
    char b[3];
    HAL::Utils::Fmt::fixed(b, v, 3);   // the shared formatter
    Lcd::write(b[0]);
    Lcd::write(b[1]);
    Lcd::write(b[2]);
}

void printHexLcd(uint8_t v) {
    char b[2];
    HAL::Utils::Fmt::hex8(b, v);       // ditto for hex
    Lcd::write(b[0]);
    Lcd::write(b[1]);
}

void repaint(uint8_t addr, uint8_t cmd) {
    Lcd::setCursor(0, 0);
    Lcd::print_P(PSTR("A:"));
    printHexLcd(addr);
    Lcd::print_P(PSTR(" C:"));
    printHexLcd(cmd);
    Lcd::print_P(PSTR("        "));
    Lcd::setCursor(0, 1);
    Lcd::print_P(PSTR("press "));
    print3(pressCount);
    Lcd::print_P(PSTR(" rpt "));
    print3(repeatCount);
}

void onIr(uint8_t addr, uint8_t cmd, bool repeatP) {
    if (repeatP) {
        // one dot per ~110 ms repeat frame: hold a button and watch
        // the cadence draw itself on the terminal
        repeatCount = static_cast<uint16_t>(repeatCount + 1);
        UART::printByte('.');
    } else {
        pressCount  = static_cast<uint16_t>(pressCount + 1);
        repeatCount = 0;
        UART::println("");             // end any dot run
        UART::print("addr ");
        UART::printHex(addr);
        UART::print(" cmd ");
        UART::printHex(cmd);
        UART::print(" ");              // dots follow on this line
    }
    repaint(addr, cmd);
}

int main() {
    HAL::Ticker::setupMSTimer();
    UART::init<9600>();
    ir.begin();
    ir.setOnCommand(&onIr);
    sei();

    UART::println_P(PSTR("ir-probe: point an NEC remote at me"));
    Lcd::begin();
    Lcd::print_P(PSTR("ir-probe"));
    Lcd::setCursor(0, 1);
    Lcd::print_P(PSTR("press a button"));

    while (1) {
        ir.process();   // callbacks (LCD + UART) fire here, main ctx
        HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
    }
}
