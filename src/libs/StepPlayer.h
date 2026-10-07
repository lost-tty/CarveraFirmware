#pragma once

#include "Block.h"

#include <stdint.h>

// Per motor a 0.32 fraction: the motor steps when it carries
struct StepPlayer {
    uint32_t left[k_max_actuators];
    uint32_t acc[k_max_actuators];
    uint32_t share[k_max_actuators];
    uint32_t made{0}, total{0};

    void reset() { made= total= 0; }

    void start(const Block &b, uint8_t n)
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
        if(acc[m] >= was)
            return false;

        --left[m];
        return true;
    }
};
