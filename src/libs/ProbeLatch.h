#pragma once

#include "ActuatorCoordinates.h"

#include <stdint.h>
#include <stdlib.h>

// The step counters at the first interrupt sample that saw the input asserted.
struct ProbeLatch {
    enum State : uint8_t { IDLE, ARMED, TRIGGERED };

    volatile State state{IDLE};
    volatile int32_t steps[k_max_actuators]{};
    volatile bool pending{false};

    void arm() { state= ARMED; pending= false; }
    void clear() { state= IDLE; pending= false; }
    void take(const int32_t now[], uint8_t n)
    {
        for (uint8_t m = 0; m < n; m++) steps[m]= now[m];
    }
    void trigger()
    {
        state= TRIGGERED;
        pending= true;
    }
};

struct WatchRule {
    uint32_t motors{0};       // bit per motor whose travel counts towards the hysteresis
    uint16_t hysteresis{0};   // steps a watched motor travels with the input asserted before it is a hit

    bool seen{false};
    volatile bool hit{false};

    void arm()
    {
        seen= false;
        hit= false;
    }

    // true on the sample that is the hit
    bool sample(bool asserted, const int32_t now[], uint8_t n, ProbeLatch &latch)
    {
        if(!asserted) {
            seen= false;
            return false;
        }
        if(!seen) {
            seen= true;
            latch.take(now, n);
        }
        for (uint8_t m = 0; m < n; m++) {
            if((motors & (1UL << m)) && abs(now[m] - latch.steps[m]) >= hysteresis) {
                hit= true;
                latch.trigger();
                return true;
            }
        }
        return false;
    }
};
