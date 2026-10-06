#pragma once

#include "mbed.h"

class SdSpi : public mbed::SPI {
public:
    using mbed::SPI::SPI;
    // out's bytes (else 0xFF) out, what comes back into in (else dropped); false: received
    // bytes were lost
    bool exchange(const char *out, char *in, int n);
};
