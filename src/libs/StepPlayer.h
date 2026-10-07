#pragma once

#include "Block.h"

#include <stdint.h>

// Per motor a 0.32 fraction: the motor steps when it carries, and on every path step when its
// share is 0, the longest motor's.
struct StepPlayer {
    uint32_t left[k_max_actuators];
    uint32_t acc[k_max_actuators];
    uint32_t share[k_max_actuators];
    uint32_t made{0}, total{0};
    uint8_t n{0};

    void reset()
    {
        made= total= 0;
        for (uint8_t m = 0; m < k_max_actuators; m++)
            left[m]= 0;
    }

    void start(const Block &b)
    {
        uint32_t longest= 0;
        for (uint8_t m = 0; m < n; m++) {
            if(b.steps[m] > longest)
                longest= b.steps[m];
        }
        for (uint8_t m = 0; m < n; m++) {
            left[m]= b.steps[m];
            share[m]= Block::share_of(b.steps[m], longest);
            acc[m]= 0x80000000UL;   // half a step in
        }
        made= 0;
        total= longest;
    }

    bool tick()
    {
        if(made >= total)
            return false;

        ++made;
        return true;
    }

    bool motor(uint8_t m)
    {
        if(left[m] == 0)
            return false;

        uint32_t was= acc[m];
        acc[m]+= share[m];
        if(share[m] != 0 && acc[m] >= was)
            return false;

        --left[m];
        return true;
    }

    bool busy(uint8_t m) const { return left[m] != 0; }

    bool idle() const
    {
        for (uint8_t m = 0; m < n; m++) {
            if(left[m] != 0)
                return false;
        }
        return true;
    }

    void drop(uint8_t m) { left[m]= 0; }
};
