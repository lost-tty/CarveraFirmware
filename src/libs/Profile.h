#pragma once

#include <stdint.h>
#include "us_ticker_api.h"

// A named stopwatch. `prof` prints calls, mean and worst case since it last asked.
class Profile {
public:
    static const uint8_t k_max= 8;
    struct Slot { const char *name; uint32_t calls, us, worst; };
    static Slot slots[k_max];
    static uint8_t used;

    explicit Profile(const char *name)
    {
        for (id= 0; id < used; id++) {
            if(slots[id].name == name) return;
        }
        if(used < k_max) {
            slots[used++].name= name;
        }
    }
    struct Scope {
        uint8_t id; uint32_t t0;
        explicit Scope(uint8_t i) : id(i), t0(us_ticker_read()) {}
        ~Scope()
        {
            if(id >= k_max) return;
            uint32_t dt= us_ticker_read() - t0;
            Slot &s= slots[id];
            s.calls++; s.us += dt;
            if(dt > s.worst) s.worst= dt;
        }
    };
    Scope scope() const { return Scope(id); }
    static void reset()
    {
        for (uint8_t i = 0; i < used; i++) {
            slots[i].calls= slots[i].us= slots[i].worst= 0;
        }
    }

private:
    uint8_t id{k_max};
};

// Once per scope: the names are fixed.
#define PROFILE(name) static Profile prof_(name); Profile::Scope prof_scope_= prof_.scope()
