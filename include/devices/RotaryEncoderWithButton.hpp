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

    bool     suppressNextRelease;
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
          suppressNextRelease {   false },
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

    void notifyInterruptOccurred(uint32_t now, uint8_t changed) {
        btn.notifyInterruptOccurred(now, changed);
        re.notifyInterruptOccurred(now, changed);
    }

    void setOnRelease(Callback fnptr)    {    onRelease = fnptr; }
    void setOnPress(Callback fnptr)      {      onPress = fnptr; }
    void setOnLongPress(Callback fnptr)  {  onLongPress = fnptr; }
    void setOnCW(Callback fnptr)         {         onCW = fnptr; }
    void setOnCCW(Callback fnptr)        {        onCCW = fnptr; }
    void setOnPressedCW(Callback fnptr)  {  onPressedCW = fnptr; }
    void setOnPressedCCW(Callback fnptr) { onPressedCCW = fnptr; }

    REWithButtonAction process() {
        ButtonAction btnAction       {        btn.process() };
        RotaryEncoderAction reAction {         re.process() };
        bool btnStableState          { btn.getStableState() };

        if (reAction == RotaryEncoderAction::CW) {
            // if pressed
            if (btnStableState != btnPassiveState) {
                suppressNextRelease = true;
                if (onPressedCW) onPressedCW();
                return REWithButtonAction::PRESSED_CW;
            }
            // not pressed
            if (onCW) onCW();
            return REWithButtonAction::CW;
        }
        if (reAction == RotaryEncoderAction::CCW) {
            if (btnStableState != btnPassiveState) {
                suppressNextRelease = true;
                if (onPressedCCW) onPressedCCW();
                return REWithButtonAction::PRESSED_CCW;
            }
            if (onCCW) onCCW();
            return REWithButtonAction::CCW;
        }
        if (btnAction == ButtonAction::LONG_PRESS) {
            if (onLongPress) onLongPress();
            return REWithButtonAction::LONG_PRESS;
        }
        if (btnAction == ButtonAction::PRESS) {
            if (onPress) onPress();
            return REWithButtonAction::PRESS;
        }
        if (btnAction == ButtonAction::RELEASE) {
            if (suppressNextRelease) {
                suppressNextRelease = false;
                return REWithButtonAction::NONE;
            }
            if (onRelease) onRelease();
            return REWithButtonAction::RELEASE;
        }
        return REWithButtonAction::NONE;
    }

    bool pendingDebounceTimeout() {
        return btn.pendingDebounceTimeout() || 
               re.pendingDebounceTimeout();
    }

};

}
}
