#pragma once

#include "common.hpp"

#include <stdint.h>
#include <util/atomic.h>
#include "gpio.hpp"

/**
 * Quadrature decoder for detented mechanical rotary encoders.
 *
 * Unlike the other devices, the decode itself runs in ISR context: on
 * every edge of either channel the ISR-side state machine folds the new
 * (CLK, DT) pair into a signed quarter-step accumulator, and process()
 * (main-loop context, as always) atomically claims whole detents from it
 * and fires the callbacks. This is the sanctioned exception to the
 * "ISRs only notify" rule -- see the README's concurrency section for
 * the reasoning and the invariants that make it safe. The short version:
 * direction lives in the *instantaneous* phase relationship of the two
 * channels, so sampling it a main-loop-latency later reads the wrong
 * phase at fast rotation and reverses the step.
 *
 * The `debounceWaitTime` and `passiveState` template parameters are
 * vestigial (kept so existing instantiations keep compiling): a Gray-code
 * table decode needs no debouncing -- contact bounce retraces adjacent
 * states and sums to zero -- and has no notion of a passive level.
 */

namespace HAL {

enum class RotaryEncoderAction : uint8_t { NONE, CW, CCW };

namespace Devices {

template<uint8_t clkPin,
         uint8_t dtPin,
         uint32_t debounceWaitTime,  // vestigial: ignored (see above)
         bool passiveState,          // vestigial: ignored (see above)
         bool usePullupP,
         bool reverseP=false,
         uint8_t stepsPerDetent=4>   // quarter-steps per detent; KY-040
                                     // style parts do one full electrical
                                     // cycle (4) per click
class RotaryEncoder {

    HAL::GPIO::GPIO<clkPin> clk;
    HAL::GPIO::GPIO<dtPin>  dt;

    // ISR-owned decode state; main loop touches quarterSteps only inside
    // atomic claim sections
    volatile uint8_t prevQuadState;
    volatile int8_t  quarterSteps;

    Callback onCW;
    Callback onCCW;

    // if the main loop stalls long enough for the accumulator to
    // approach the int8_t rails, drop steps instead of wrapping:
    // a wrap would replay as ~32 phantom detents in the wrong direction
    static constexpr int8_t SATURATION_LIMIT { 120 };

    uint8_t readQuadState() {
        return static_cast<uint8_t>((clk.read() ? 2 : 0) |
                                    (dt.read()  ? 1 : 0));
    }

  public:
    RotaryEncoder()
        : clk           {         },
          dt            {         },
          prevQuadState {       3 }, // detent rest: both channels high
          quarterSteps  {       0 },
          onCW          { nullptr },
          onCCW         { nullptr } {
    }

    void begin() {
        if constexpr (usePullupP) {
            clk.setInputPullup();
            dt.setInputPullup();
        }
        else {
            clk.setInput();
            dt.setInput();
        }
        // both channels interrupt: every quarter-step reaches the table,
        // in contrast to the old decode-on-CLK-only scheme
        clk.enablePCINT();
        dt.enablePCINT();
        prevQuadState = readQuadState();
    }

    // ISR context. The timestamp is unused -- the decoder is edge-driven,
    // not timed -- but the signature matches the other devices so
    // composites can fan out uniformly
    void notifyInterruptOccurred(uint32_t /* now */, HAL::GPIO::Port port,
                                 uint8_t changed) {
        // port comparisons are against constexprs and fold away; the two
        // checks stay separate because CLK and DT may legitimately live
        // on different ports
        bool ours { false };
        if (port == clk.info.port && (changed & clk.mask)) ours = true;
        if (port == dt.info.port  && (changed & dt.mask))  ours = true;
        if (!ours) return;

        // Gray-code transition table, indexed by (prev << 2) | current
        // where a state is (CLK << 1) | DT. One entry per transition:
        //   +1 : one quarter-step clockwise
        //   -1 : one quarter-step counterclockwise
        //    0 : no change, or both channels flipped at once (a missed
        //        intermediate state -- direction unknowable, so no step
        //        is safer than a guessed step)
        // CW is the cycle 11 -> 10 -> 00 -> 01 -> 11, matching the
        // direction convention of the previous sample-DT-on-CLK-fall
        // decoder so existing wiring keeps its meaning
        static constexpr int8_t QUAD_DELTAS[16] = {
             0, +1, -1,  0,
            -1,  0,  0, +1,
            +1,  0,  0, -1,
             0, -1, +1,  0
        };

        // read the live pins rather than trusting `changed` history:
        // if latency merged two edges, the table sees the 2-bit jump and
        // emits 0 -- one quarter-step lost, but never a reversed one,
        // and the state is resynchronized to reality immediately
        uint8_t newState { readQuadState() };
        int8_t  delta    { QUAD_DELTAS[static_cast<uint8_t>(
                               (prevQuadState << 2) | newState)] };
        prevQuadState = newState;

        if (delta != 0) {
            int8_t acc { quarterSteps };
            if (acc < SATURATION_LIMIT && acc > -SATURATION_LIMIT) {
                quarterSteps = static_cast<int8_t>(acc + delta);
            }
        }
    }

    void setOnCW(Callback fnptr)  {  onCW = fnptr; }
    void setOnCCW(Callback fnptr) { onCCW = fnptr; }

    // main-loop context: claim at most one whole detent per call (the
    // loop runs at kHz rates, far faster than fingers; anything banked
    // beyond one detent drains over the next few iterations). Bounce and
    // direction reversals mid-cycle cancel arithmetically in the
    // accumulator before ever reaching a callback
    RotaryEncoderAction process() {
        int8_t detents { 0 };
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (quarterSteps >= stepsPerDetent) {
                quarterSteps = static_cast<int8_t>(quarterSteps
                                                   - stepsPerDetent);
                detents = 1;
            }
            else if (quarterSteps <= -stepsPerDetent) {
                quarterSteps = static_cast<int8_t>(quarterSteps
                                                   + stepsPerDetent);
                detents = -1;
            }
        }
        if (detents == 0) return RotaryEncoderAction::NONE;

        bool cwP { detents > 0 };
        if constexpr (reverseP) cwP = !cwP;

        if (cwP) {
            if (onCW) onCW();
            return RotaryEncoderAction::CW;
        }
        if (onCCW) onCCW();
        return RotaryEncoderAction::CCW;
    }

    // kept for sleep-gate compatibility. The decoder has no timed state
    // (nothing here reads the Ticker), so "pending" now means a full
    // detent is banked but unclaimed -- don't power down before the main
    // loop has rendered its effect. A partial cycle is not pending: its
    // remaining edges will wake the MCU themselves
    bool pendingDebounceTimeout() {
        int8_t qs { quarterSteps };  // single-byte volatile read: atomic
        return qs >= stepsPerDetent || qs <= -stepsPerDetent;
    }

};



}
}
