#pragma once

#include "common.hpp"

#include <stdint.h>

#include "../ticker.hpp"

/**
 * Tick-less cooperative scheduler: a task table that TELLS YOU HOW
 * LONG TO SLEEP.
 *
 * The superloop idiom (`if (now - last >= period)`) is a fine
 * scheduler at one or two cadences; by three or four it degenerates
 * into a pile of lastX variables. This is that idiom, tabled -- with
 * one addition that makes it avril-shaped: run() executes whatever is
 * due and returns the milliseconds until the NEXT deadline, so the
 * loop bottom can sleep exactly that long instead of spinning at the
 * ticker's 1 kHz:
 *
 *     while (1) {
 *         uint32_t wait { sched.run() };
 *         if (wait >= 300) {
 *             // long gap: PWR_DOWN-nap it away. sleepFor pauses the
 *             // ticker and credits the slept time back, so the
 *             // scheduler's timeline sails through the nap
 *             HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS256>(1);
 *         } else {
 *             HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
 *         }
 *     }
 *
 * Semantics, stated so they're decisions rather than surprises:
 *   - a task's period restarts when the task RUNS, and there is no
 *     catch-up: if the loop stalls past two deadlines, the task runs
 *     once, not twice. Cadences are "at most this often, roughly this
 *     often" -- right for sampling and display, wrong for counting.
 *   - tasks run in main context, in add() order, to completion:
 *     cooperative means a slow task delays its neighbors. Keep tasks
 *     short; anything long belongs in the loop proper.
 *   - timing rides on getNumTicks(): wrap-safe (~49.7 days), and only
 *     as accurate as what feeds it -- watchdog-napped time is
 *     credited at nominal +/-10%, which is exactly why a logger
 *     should timestamp records from an RTC, not from this.
 *   - tasks are due immediately after add(): the first pass through
 *     run() executes everything once (boot sample for free).
 */

namespace HAL {
namespace Utils {

template<uint8_t maxTasks>
class Scheduler {

    struct Task {
        Callback fn       { nullptr };
        uint32_t periodMs { 0 };
        uint32_t lastRun  { 0 };
    };

    Task    tasks[maxTasks];
    uint8_t count;

  public:
    Scheduler() : tasks {}, count { 0 } {}

    bool add(Callback fn, uint32_t periodMs) {
        if (count >= maxTasks || fn == nullptr || periodMs == 0) {
            return false;
        }
        tasks[count].fn       = fn;
        tasks[count].periodMs = periodMs;
        // backdated one period: due on the first run() pass
        tasks[count].lastRun  = HAL::Ticker::getNumTicks() - periodMs;
        ++count;
        return true;
    }

    // run everything due; return ms until the soonest next deadline
    // (the loop's sleep allowance). An empty table returns 1000 so a
    // misconfigured loop still wakes
    uint32_t run() {
        uint32_t soonest { static_cast<uint32_t>(-1) };
        for (uint8_t i = 0; i < count; ++i) {
            uint32_t now     { HAL::Ticker::getNumTicks() };
            uint32_t elapsed { now - tasks[i].lastRun };   // wrap-safe
            if (elapsed >= tasks[i].periodMs) {
                tasks[i].lastRun = now;   // restart period; no catch-up
                tasks[i].fn();
                elapsed = 0;
            }
            uint32_t remain { tasks[i].periodMs - elapsed };
            if (remain < soonest) soonest = remain;
        }
        return (count == 0) ? 1000UL : soonest;
    }

};

}
}
