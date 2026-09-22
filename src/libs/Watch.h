#pragma once

#include "PinGroup.h"
#include "ActuatorCoordinates.h"

#include <stdint.h>

// Terminates a move when an input asserts.
struct Watch {
    PinGroup inputs;
    uint32_t motors{0};
    uint16_t hysteresis{1};   // in step ticks

    uint16_t count{0};
    volatile bool    hit{false};
    volatile int32_t at_steps[k_max_actuators]{};

    void arm() { count = 0; hit = false; }
};
