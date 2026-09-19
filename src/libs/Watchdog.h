#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <stdint.h>

#include "Module.h"
#include "SoftTimer.h"

typedef enum
{
    WDT_MRI,
    WDT_RESET,
} WDT_ACTION;

class Watchdog : public Module {
public:
    Watchdog(uint32_t timeout, WDT_ACTION action)
        : feed_timer("Watchdog", FEED_MS, true, this, &Watchdog::tick),
          timeout(timeout), action(action) {}

    void arm();
    void feed();

    void on_module_loaded();

    // the main task reports in, from the loop or from inside a blocking wait
    void alive() { main_loop_alive = true; }

    void configure(uint32_t new_timeout, WDT_ACTION new_action) {
        timeout = new_timeout;
        action = new_action;
    }

private:
    static const uint32_t FEED_MS = 1000;   // well under the 10 s timeout

    void tick();

    SoftTimer feed_timer;
    volatile bool main_loop_alive = false;
    uint32_t timeout;
    WDT_ACTION action;
};

extern Watchdog watchdog;

#endif /* _WATCHDOG_H */
