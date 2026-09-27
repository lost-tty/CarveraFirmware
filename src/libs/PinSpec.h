#pragma once

#include <cstdint>
#include <cstddef>

// A pin as stored in the config: the parsed form of a spec such as "2.13!^", in 2 bytes.
// Pin::from_spec() performs the hardware setup.
namespace PinSpec {
    enum : uint16_t {
        PORT = 0x0007, PIN_SHIFT = 3, PIN = 0x00f8,
        INVERT = 0x0100, OPEN_DRAIN = 0x0200,
        PULL_SHIFT = 10, PULL = 0x0c00, // 0 unchanged, 1 up (^), 2 down (v), 3 none (-).
        REPEATER = 0x1000,
        CONNECTED = 0x8000,             // Clear for "nc".
    };

    uint16_t parse(const char *text);
    void format(uint16_t spec, char *buf, size_t bufsize);

    inline bool connected(uint16_t s) { return s & CONNECTED; }
    inline int port(uint16_t s) { return s & PORT; }
    inline int pin(uint16_t s) { return (s & PIN) >> PIN_SHIFT; }
    inline int pull(uint16_t s) { return (s & PULL) >> PULL_SHIFT; }
}
