#pragma once

#include "PinGroup.h"
#include "ActuatorCoordinates.h"

#include <stdint.h>

// Terminates a move when an input asserts.
struct Watch {
    PinGroup inputs;
    uint32_t motors{0};
    uint16_t hysteresis{0};   // steps a watched motor travels with the input held before it counts

    bool seen{false};
    volatile bool    hit{false};
    volatile int32_t at_steps[k_max_actuators]{};

    void arm() { seen = false; hit = false; }
};
