#pragma once

#include "PinGroup.h"
#include "ProbeLatch.h"

#include <stdint.h>

// Terminates a move when an input asserts.
struct Watch : WatchRule {
    PinGroup inputs;
    PinGroup witness;         // sampled at the first edge into witnessed
    bool observe{false};      // record the hit, let the move run out

    volatile bool witnessed{false};

    void arm()
    {
        WatchRule::arm();
        witnessed= false;
    }
};
