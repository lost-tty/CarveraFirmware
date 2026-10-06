#ifndef WIFIPEER_H
#define WIFIPEER_H

#include "Endpoint.h"

#include <cstddef>
#include <cstdint>

class TxSource;

// A client's output: a ring its writers fill with whole frames and the wifi task empties, and
// a download frame that goes out once the bytes written before it have.
struct TxRing {
    uint8_t* buf = nullptr;              // WIFI_TX_RING bytes, from the heap on first use
    volatile uint32_t head = 0;          // bytes written in all
    volatile uint32_t tail = 0;          // bytes sent in all, the task only
    TxSource* volatile source = nullptr;
    uint32_t source_at = 0;              // the head when it came
    size_t source_sent = 0;
    volatile uint32_t moved_at = 0;      // a tick: when the client last took something
    void* writer = nullptr;              // the task in a write to it: one at a time
    volatile uint32_t dropped = 0;       // frames that found no room
};

// A client as the wifi link sees it: where its bytes go.
struct WifiPeer {
    Endpoint who;
    uint8_t link = 0;
    volatile uint32_t rx_bytes = 0, tx_bytes = 0;   // since it connected, for wifi clients
    TxRing tx;

    bool live() const { return who.port != 0; }
    bool is(const uint8_t ip[4], uint16_t port) const { return who == Endpoint(ip, port); }
};

#endif
