#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "../utils/IntTransitionDebouncer.hpp"

/*
 *  TODO  test with pulldowns, opposite passiveState
 *        and see if direction is correct
 */

/*
 *  TODO  add fast rotate detection?
 */

/*
 *  TODO  add documentation
 */

namespace HAL {

enum class RotaryEncoderAction : uint8_t { NONE, CW, CCW };

namespace Drivers {

template<uint8_t clkPin,
         uint8_t dtPin,
         uint32_t debounceWaitTime,
         bool passiveState,
         bool usePullupP,
         bool reverseP=false>
class RotaryEncoder {

    using Callback = void (*)();

    HAL::GPIO::GPIO<dtPin> dt;
    HAL::Utils::IntTransitionDebouncer<clkPin,
                                       debounceWaitTime,
                                       passiveState,
                                       usePullupP> debouncer;
    Callback onCW;
    Callback onCCW;
    
    //  TODO  try inline
    // RotaryEncoderAction doCW() {
    //     if (onCW) onCW();
    //     return RotaryEncoderAction::CW;
    // }
    // RotaryEncoderAction doCCW() {
    //     if (onCW) onCCW();
    //     return RotaryEncoderAction::CW;
    // }

  public:
    RotaryEncoder()
        : dt        { },
          debouncer { },
          onCW      { nullptr },
          onCCW     { nullptr } {
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

    void setOnCW(Callback fnptr)  {  onCW = fnptr; }
    void setOnCCW(Callback fnptr) { onCCW = fnptr; }


    RotaryEncoderAction process() {
        Transition dbTransition { debouncer.processAnyInterrupts() };
        RotaryEncoderAction rea { RotaryEncoderAction::NONE };

        if (dbTransition == Transition::FALLING) {
            bool dtState = dt.read();
            if (dtState == passiveState) {
                rea = (!reverseP) ? RotaryEncoderAction::CCW : RotaryEncoderAction::CW;
            } else {
                rea = (!reverseP) ? RotaryEncoderAction::CW : RotaryEncoderAction::CCW;
            }
        }

        switch (rea) {
            case RotaryEncoderAction::CW:
                if (onCW) onCW();
                return RotaryEncoderAction::CW;
                break;
            case RotaryEncoderAction::CCW:
                if (onCCW) onCCW();
                return RotaryEncoderAction::CCW;
                break;
            default:
                return rea;
        }
    }


    bool pendingDebounceTimeout() {
        return debouncer.pendingDebounceTimeout();
    }

};
             


}
}
