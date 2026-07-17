#pragma once

#include "common.hpp"

#include <stdint.h>
#include <util/atomic.h>

#include "../gpio.hpp"
#include "../ticker.hpp"

/**
 * NEC infrared remote decoder for TSOP4838-style receivers.
 *
 * The TSOP is a complete IR front-end in a can: photodiode, AGC,
 * 38 kHz bandpass, demodulator. What reaches the pin is a clean
 * digital signal -- idle HIGH (internal ~30k pull-up), LOW while
 * carrier is detected -- so "receiving IR" reduces to measuring pulse
 * timing on one input. See TSOP4838.md at the repo root for the full
 * design study (jitter budget, protocol tables, RAM math, sleep
 * interaction); this header implements exactly that design.
 *
 * THE CENTRAL TRICK: decode on falling edges only. Every NEC element
 * begins with a mark, so consecutive falling edges bracket exactly
 * one (mark + space) pair, and the four possible periods separate
 * cleanly -- 13500 us frame header, 11250 us repeat header, 2250 us
 * "1", 1125 us "0". If the TSOP's AGC stretches a mark (it can, by
 * ~150 us), it steals exactly that much from the following space:
 * the falling-to-falling SUM is unchanged, and the receiver's
 * largest analog distortion cancels by construction. Rising edges
 * are checked and discarded in a few cycles (PCINT can't select an
 * edge).
 *
 * THE CONTRACT NOTE: this is the second resident of the sanctioned
 * ISR-side-decoder exception (the RotaryEncoder is the first). The
 * decode state machine runs in notifyInterruptOccurred() -- ISR
 * context -- because the contract-pure alternative (buffer 34 edge
 * timestamps, classify in process()) costs a 68+ byte buffer that a
 * tiny85 with ~128 free bytes cannot spare, versus ~50 cycles of
 * classification per edge, which it easily can. The contract's
 * invariants hold: decoder state is touched ONLY in ISR context,
 * completed frames cross via one atomic claim in process(), and
 * callbacks fire only from process(), in main context.
 *
 * TIMESTAMPS ARE MICROSECONDS: notifyInterruptOccurred takes
 * HAL::Ticker::getMicros(), not getNumTicks() -- the millisecond
 * ticker cannot tell 1125 us from 2250 us. In a shared ISR, take the
 * getMicros() timestamp FIRST, before other devices' notify work, so
 * their processing time never becomes IR timestamp jitter. And after
 * a PWR_DOWN wake, call Ticker::resume() before reading it.
 *
 * THE SLEEP GATE: a frame in flight has multi-ms quiet gaps that a
 * sleep-happy main loop would happily power down into, freezing the
 * us clock and garbling every subsequent period. Add busy() to the
 * power-down gate alongside pendingDebounceTimeout(). A lone noise
 * edge cannot wedge busy() on: state older than ~20 ms (no NEC frame
 * has a gap that long) is declared dead and reset.
 *
 * Frame layout (LSB-first): address, ~address, command, ~command.
 * The command pair is ALWAYS validated (corruption is discarded, not
 * acted on). The address check is optional via template params,
 * because extended-NEC remotes use the two address bytes as one
 * 16-bit address with no complement.
 *
 * Repeats: a held button sends the full frame once, then a short
 * repeat frame every ~110 ms. These are reported with isRepeat=true
 * and the last known command -- exactly what press-and-hold ramping
 * wants. A repeat arriving with NO prior command (cold start, first
 * frame corrupted) is dropped.
 */

namespace HAL {
namespace Devices {

template<uint8_t irPin,
         bool    validateAddress = false,
         uint8_t expectedAddress = 0>
class IRReceiverNEC {

    using CommandCallback = void (*)(uint8_t address, uint8_t command,
                                     bool isRepeat);

    // acceptance windows, ~+/-20-25% around nominal (TSOP4838.md §4).
    // Bench-tune against real remotes before trusting edge cases
    static constexpr uint32_t BIT0_MIN   {   900 };  // nominal 1125
    static constexpr uint32_t BIT0_MAX   {  1350 };
    static constexpr uint32_t BIT1_MIN   {  1900 };  // nominal 2250
    static constexpr uint32_t BIT1_MAX   {  2600 };
    static constexpr uint32_t RPT_MIN    { 10200 };  // nominal 11250
    static constexpr uint32_t RPT_MAX    { 11900 };
    static constexpr uint32_t HDR_MIN    { 12000 };  // nominal 13500
    static constexpr uint32_t HDR_MAX    { 15000 };
    static constexpr uint32_t STALE_US   { 20000 };  // no in-frame gap
                                                     // is this long

    enum class State : uint8_t { IDLE, SYNC, RECEIVING };

    HAL::GPIO::GPIO<irPin> gpio;

    // ISR-owned decode state: written only in ISR context
    volatile State    state;
    volatile uint32_t lastFallUs;
    volatile uint32_t bits;
    volatile uint8_t  bitCount;

    // the completed-frame latch: written by the ISR, claimed
    // atomically by process()
    volatile bool     framePendingP;
    volatile uint8_t  latchedAddr;
    volatile uint8_t  latchedCmd;
    volatile bool     latchedRepeatP;
    volatile bool     haveLastP;   // repeat hygiene: is there a
                                   // command to repeat?

    CommandCallback onCommand;

  public:
    IRReceiverNEC()
        : gpio           {              },
          state          { State::IDLE  },
          lastFallUs     {            0 },
          bits           {            0 },
          bitCount       {            0 },
          framePendingP  {        false },
          latchedAddr    {            0 },
          latchedCmd     {            0 },
          latchedRepeatP {        false },
          haveLastP      {        false },
          onCommand      {      nullptr } {
    }

    void begin() {
        gpio.setInput();   // the TSOP drives push-pull-ish with its
                           // own pull-up: no internal pullup needed
        gpio.enablePCINT();
    }

    void setOnCommand(CommandCallback fnptr) { onCommand = fnptr; }

    // ISR context. Pass HAL::Ticker::getMicros() -- microseconds, not
    // ticks -- taken as early in the ISR as possible
    void notifyInterruptOccurred(uint32_t nowMicros,
                                 HAL::GPIO::Port port,
                                 uint8_t changed) {
        if (port != gpio.info.port) return;
        if (!(changed & gpio.mask)) return;
        if (gpio.read()) return;   // rising edge: not ours to decode

        uint32_t period { nowMicros - lastFallUs };  // wrap-safe
        lastFallUs = nowMicros;

        switch (state) {
            case State::IDLE:
                // this edge only establishes the reference point
                state = State::SYNC;
                return;

            case State::SYNC:
                if (period >= HDR_MIN && period <= HDR_MAX) {
                    state    = State::RECEIVING;
                    bits     = 0;
                    bitCount = 0;
                }
                else if (period >= RPT_MIN && period <= RPT_MAX) {
                    // repeat header: report the last known command --
                    // if there is one to repeat
                    if (haveLastP) {
                        latchedRepeatP = true;
                        framePendingP  = true;
                    }
                    state = State::SYNC;   // this edge re-syncs
                }
                // anything else: noise or a foreign protocol --
                // restart SYNC from this edge (already timestamped)
                return;

            case State::RECEIVING: {
                bool bitVal;
                if      (period >= BIT0_MIN && period <= BIT0_MAX) {
                    bitVal = false;
                }
                else if (period >= BIT1_MIN && period <= BIT1_MAX) {
                    bitVal = true;
                }
                else {
                    // protocol violation: abandon, re-sync from here
                    state = State::SYNC;
                    return;
                }
                // NEC is LSB-first: shift in from the top so the
                // finished word reads addr / ~addr / cmd / ~cmd
                // from its low byte up
                bits = (bits >> 1)
                     | (bitVal ? 0x80000000UL : 0UL);
                bitCount = static_cast<uint8_t>(bitCount + 1);
                if (bitCount < 32) return;

                // 32nd bit: validate and latch. Command complement is
                // mandatory (a corrupted frame is DISCARDED, never
                // misread); address equality is opt-in because
                // extended NEC repurposes the address complement
                uint32_t w       { bits };
                uint8_t  addr    { static_cast<uint8_t>(w) };
                uint8_t  cmd     { static_cast<uint8_t>(w >> 16) };
                uint8_t  invCmd  { static_cast<uint8_t>(w >> 24) };
                bool okP { cmd == static_cast<uint8_t>(~invCmd) };
                if (validateAddress && addr != expectedAddress) {
                    okP = false;
                }
                if (okP) {
                    latchedAddr    = addr;
                    latchedCmd     = cmd;
                    latchedRepeatP = false;
                    haveLastP      = true;
                    framePendingP  = true;
                }
                state = State::IDLE;   // trailer mark's edge will
                                       // re-establish sync
                return;
            }
        }
    }

    // main-loop context: claim a completed frame atomically, then run
    // the callback outside the atomic section
    void process() {
        uint8_t addr;
        uint8_t cmd;
        bool    repeatP;
        bool    haveP { false };
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (framePendingP) {
                framePendingP = false;
                addr    = latchedAddr;
                cmd     = latchedCmd;
                repeatP = latchedRepeatP;
                haveP   = true;
            }
        }
        if (haveP && onCommand) onCommand(addr, cmd, repeatP);
    }

    // true while a frame is mid-flight: the power-down sleep gate
    // must check this, or the us clock freezes mid-frame and every
    // period in it turns to garbage. Self-healing: state older than
    // STALE_US (no legal NEC gap is that long) is declared dead here,
    // so a lone noise edge can't wedge the gate on and quietly ruin
    // a battery
    bool busy() {
        bool busyP { false };
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (state != State::IDLE) {
                uint32_t now { HAL::Ticker::getMicros() };
                if (now - lastFallUs > STALE_US) {
                    state = State::IDLE;   // dead air: noise edge
                } else {
                    busyP = true;
                }
            }
        }
        return busyP;
    }

};

}
}
