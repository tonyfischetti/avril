#pragma once

#include "common.hpp"

#include <stdint.h>
#include "RotaryEncoder.hpp"
#include "Button.hpp"

namespace HAL {

enum REWithButtonAction : uint8_t {
    NONE,
    RELEASE,
    PRESS,
    LONG_PRESS,
    CW,
    CCW,
    PRESSED_CW,
    PRESSED_CCW
};

namespace Devices {

template<uint8_t btnPin,
         uint32_t btnDebounceWaitTime,
         uint32_t btnLongPressWaitTime,
         bool     btnPassiveState,
         bool     btnUsePullup,
         uint8_t  reClkPin,
         uint8_t  reDtPin,
         uint32_t reDebounceWaitTime,
         bool     rePassiveState,
         bool     reUsePullup,
         bool     btnSuppressReleaseAfterLongPress=true,
         bool     btnAllowConsecutiveLongPresses=false,
         bool     reReverseP=false>
class RotaryEncoderWithButton {

    HAL::Devices::Button<btnPin,
                         btnDebounceWaitTime,
                         btnLongPressWaitTime,
                         btnPassiveState,
                         btnUsePullup,
                         btnSuppressReleaseAfterLongPress,
                         btnAllowConsecutiveLongPresses> btn;

    HAL::Devices::RotaryEncoder<reClkPin,
                                reDtPin,
                                reDebounceWaitTime,
                                rePassiveState,
                                reUsePullup,
                                reReverseP> re;

    Callback onRelease;
    Callback onPress;
    Callback onLongPress;
    Callback onCW;
    Callback onCCW;
    Callback onPressedCW;
    Callback onPressedCCW;

  public:
    RotaryEncoderWithButton()
        : btn                 {         },
          re                  {         },
          onRelease           { nullptr },
          onPress             { nullptr },
          onLongPress         { nullptr },
          onCW                { nullptr },
          onCCW               { nullptr },
          onPressedCW         { nullptr },
          onPressedCCW        { nullptr } {
    }

    void begin() {
        btn.begin();
    re.begin();
    }

    // fans out to both children; each filters by its own port, so the
    // button and the encoder may legitimately live on different ports
    // (relevant on the ATmega328P, which has three PCINT vectors)
    void notifyInterruptOccurred(uint32_t now, HAL::GPIO::Port port,
                                 uint8_t changed) {
        btn.notifyInterruptOccurred(now, port, changed);
        re.notifyInterruptOccurred(now, port, changed);
    }

    void setOnRelease(Callback fnptr)    {    onRelease = fnptr; }
    void setOnPress(Callback fnptr)      {      onPress = fnptr; }
    void setOnLongPress(Callback fnptr)  {  onLongPress = fnptr; }
    void setOnCW(Callback fnptr)         {         onCW = fnptr; }
    void setOnCCW(Callback fnptr)        {        onCCW = fnptr; }
    void setOnPressedCW(Callback fnptr)  {  onPressedCW = fnptr; }
    void setOnPressedCCW(Callback fnptr) { onPressedCCW = fnptr; }

    REWithButtonAction process() {
        // button events take priority over rotary detents, and the order
        // matters: the encoder *banks* its detents, so one deferred to
        // the next loop iteration (kHz rates) loses nothing -- but a
        // button event is consumed from the debouncer the moment
        // btn.process() returns it, so preferring the rotary action (as
        // this used to) silently dropped any PRESS/RELEASE that happened
        // to coincide with a banked detent
        ButtonAction btnAction { btn.process() };
        if (btnAction == ButtonAction::LONG_PRESS) {
            if (onLongPress) onLongPress();
            return REWithButtonAction::LONG_PRESS;
        }
        if (btnAction == ButtonAction::PRESS) {
            if (onPress) onPress();
            return REWithButtonAction::PRESS;
        }
        if (btnAction == ButtonAction::RELEASE) {
            // suppression of the release that ends a chorded gesture
            // already happened inside the Button (see
            // suppressNextReleaseEvent below); a RELEASE that reaches
            // this point is a genuine click
            if (onRelease) onRelease();
            return REWithButtonAction::RELEASE;
        }

        RotaryEncoderAction reAction { re.process() };
        if (reAction == RotaryEncoderAction::NONE) {
            return REWithButtonAction::NONE;
        }

        bool pressedP { btn.getStableState() != btnPassiveState };
        if (pressedP) {
            // the chord's release must not also fire onRelease. The
            // suppression flag lives in the Button -- its long-press
            // logic arms the same flag, and keeping a second copy here
            // caused a bug: a chord held past the long-press threshold
            // armed both, one release cleared only the Button's, and the
            // stale composite flag ate the next ordinary click's release
            btn.suppressNextReleaseEvent();
        }

        if (reAction == RotaryEncoderAction::CW) {
            if (pressedP) {
                if (onPressedCW) onPressedCW();
                return REWithButtonAction::PRESSED_CW;
            }
            if (onCW) onCW();
            return REWithButtonAction::CW;
        }
        if (pressedP) {
            if (onPressedCCW) onPressedCCW();
            return REWithButtonAction::PRESSED_CCW;
        }
        if (onCCW) onCCW();
        return REWithButtonAction::CCW;
    }

    bool pendingDebounceTimeout() {
        return btn.pendingDebounceTimeout() || 
               re.pendingDebounceTimeout();
    }

};

}
}
