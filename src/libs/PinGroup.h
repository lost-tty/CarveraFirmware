#pragma once

#include "Pin.h"

// Input pins tested one port at a time instead of one pin at a time.
class PinGroup {
public:
    void clear() { n= 0; }

    void add(const Pin &p, bool inverted = false)
    {
        if(!p.connected()) return;
        for (uint8_t i = 0; i < n; ++i) {
            if(ports[i] != p.port) continue;
            claim(i, p, inverted);
            return;
        }
        if(n == MAX_PORTS) return;
        ports[n] = p.port;
        mask[n] = 0;
        expected[n] = 0;
        claim(n, p, inverted);
        n++;
    }

    bool any() const
    {
        for (uint8_t i = 0; i < n; ++i) {
            if((ports[i]->FIOPIN ^ expected[i]) & mask[i]) return true;
        }
        return false;
    }

    bool empty() const { return n == 0; }

    bool holds(const Pin &p) const
    {
        for (uint8_t i = 0; i < n; ++i) {
            if(ports[i] == p.port && (mask[i] & (1UL << p.pin))) return true;
        }
        return false;
    }

private:
    void claim(uint8_t i, const Pin &p, bool inverted)
    {
        mask[i] |= 1UL << p.pin;
        if(p.is_inverting() != inverted) expected[i] |= 1UL << p.pin;
    }

    static const uint8_t MAX_PORTS = 5;

    LPC_GPIO_TypeDef *ports[MAX_PORTS]{};
    uint32_t mask[MAX_PORTS]{};
    uint32_t expected[MAX_PORTS]{};
    uint8_t  n{0};
};
