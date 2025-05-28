#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "../utils/IntTransitionDebouncer.hpp"

/*
 *  TODO  test with pulldowns, opposite passiveState
 *        and see if direction is correct
 */

namespace HAL {

enum class RotaryEncoderAction : uint8_t { NONE, CW, CCW };

namespace Drivers {

template<uint8_t clkPin,
         uint8_t dtPin,
         uint32_t debounceWaitTime,
         bool passiveState,
         bool usePullupP>
class RotaryEncoder {

    using Callback = void (*)();

    // HAL::GPIO::GPIO<clkPin> clk;
    HAL::GPIO::GPIO<dtPin> dt;
    HAL::Utils::IntTransitionDebouncer<clkPin,
                                       debounceWaitTime,
                                       passiveState,
                                       usePullupP> debouncer;
    
    //  TODO  add callbacks

  public:
    RotaryEncoder()
        : dt {},
          debouncer {} {
    }

    void begin() {
        debouncer.begin();
        if constexpr (usePullupP)
            dt.setInputPullup();
        else
            dt.setInput();
    }

    void notifyInterruptOccurred(uint32_t now, uint8_t changed) {
        debouncer.notifyInterruptOccurred(now, changed);
    }

    // void setOnCW(Callback fnptr)  { onCW = fnptr; }
    // void setOnCCW(Callback fnptr) { onCCW  = fnptr; }


    RotaryEncoderAction process() {
        Transition dbTransition { debouncer.processAnyInterrupts() };
        // if (dbTransition != Transition::NONE) { only falling!
        if (dbTransition == Transition::FALLING) {
            bool dtState = dt.read();
            if (dtState==passiveState) {
                return RotaryEncoderAction::CCW;
            }
            return RotaryEncoderAction::CW;
        }
        return RotaryEncoderAction::NONE;
    }


    bool pendingDebounceTimeout() {
        return debouncer.pendingDebounceTimeout();
    }

};
             


}
}
