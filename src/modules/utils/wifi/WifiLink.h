#ifndef WIFILINK_H
#define WIFILINK_H

#include "M8266WIFIDrv.h"
#include "WifiPeer.h"
#include "libs/StreamOutput.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstddef>

namespace mbed { class InterruptIn; }

#define WIFI_DATA_MAX_SIZE 1460
#define WIFI_DATA_TIMEOUT_MS 10
#define WIFI_POLL_MS 2                   // the task's look at the module when nothing woke it
#define WIFI_TX_RING 1024                // a client's output not yet sent; a frame finding no room
                                         // is dropped whole
#define WIFI_TX_STUCK_MS 200             // a writer waits for room only while its client took
                                         // something this recently
#define WIFI_TX_STALE_MS 2000            // a client the module takes nothing for this long loses
                                         // what waits for it
#define WIFI_TX_PEERS 4                  // clients the task serves in turn
#define WIFI_RX_SINK_MAX 4096            // one receive while a sink is attached, at most
#define WIFI_RX_SINKS 4                  // clients whose bytes go to a sink at once (uploads)
#define WIFI_REST_MS 50                  // the longest the task works through without a tick's
                                         // sleep, so the main loop runs even below it

// Every driver call holds it: the wifi task and the main loop share the module.
class ModuleLock {
public:
    ModuleLock();
    ~ModuleLock();
    ModuleLock(const ModuleLock&) = delete;
    ModuleLock& operator=(const ModuleLock&) = delete;
};

template <typename F>
auto locked(F f) -> decltype(f())
{
    ModuleLock lock;
    return f();
}

// What the wifi task received, handed to the main loop and handed back when read.
struct WifiRxBuf {
    uint8_t link;
    uint8_t ip[4];
    uint16_t port;
    uint16_t len;
    uint16_t off;                        // taken by the main loop so far
    uint8_t data[WIFI_DATA_MAX_SIZE];
};

// The wifi task: it alone receives from the module and sends the clients' output. It knows
// clients only as peers, links and endpoints; what their bytes mean is the main loop's.
class WifiLink {
public:
    // pin: high while the module holds data, or nullptr to ask the module instead
    void start(uint8_t console_link, mbed::InterruptIn* pin);
    bool running() const { return task != nullptr; }

    // What arrived, one buffer at a time: the main loop reads it and hands it back.
    WifiRxBuf* rx_head();
    void rx_release();

    // A client's output, a whole frame per write: queued, or dropped when its ring has no room
    // and the client takes nothing. Never blocks for long, never called from the wifi task.
    int write(WifiPeer* p, const uint8_t* data, size_t len);
    // a download frame, after what was written before it; false: one is still going out
    bool write_source(WifiPeer* p, TxSource* src);
    // a client's slot is someone else's from now on: what waits for the old one goes
    void forget(WifiPeer* p);

    // A transfer's input: from attach on, the peer's bytes go to the sink as the task receives
    // them, what the shared buffer still holds for it first (see StreamOutput).
    bool attach(WifiPeer* p, RxSink* sink);
    void detach(WifiPeer* p);
    bool has_sink(const WifiPeer* p) const;

    // For the main loop's own servers (WebServer), which wait for their sends themselves.
    u16 send_to_client(const u8 ip[4], u16 port, u8 link, const u8* data, size_t len);

private:
    static void task_entry(void* self);
    void run();
    void receive();
    void send_requested();
    bool send_peer(WifiPeer* p);
    void discard(WifiPeer* p);
    bool serve(WifiPeer* p);
    static void give(void* from, u16 at, u8* piece, u8 n);
    static u16 send_locked(const char* ip_str, u16 port, u8 link, const u8* data,
                           TxSource* src, size_t at, u16 len, u16* status);
    void answer_sink(const WifiRxBuf& b, u8 link);
    void on_pin_rise();
    void rest_if_due();
    void wake_starved();

    // Inside a receive, once its head named the sender: the shared buffer, or no memory for a
    // peer with a sink, whose chunks then go to take_rx.
    static u8* pick_rx(void* self, u8 link, const u8 ip[4], u16 port, u16 n);
    static void take_rx(void* self, const u8* chunk, u16 n);

    TaskHandle_t task = nullptr;
    mbed::InterruptIn* pin = nullptr;
    uint8_t console_link = 0;
    volatile bool rx_ready = false;     // the buffer holds what the main loop has not taken
    // set and cleared under ModuleLock, read by the task inside a receive
    struct { WifiPeer* peer; RxSink* sink; } sinks[WIFI_RX_SINKS] = {};
    uint8_t* rx_landing = nullptr;      // the shared buffer of the receive under way, task only
    RxSink* rx_sink = nullptr;          // or the sink its chunks go to
    WifiPeer* rx_peer = nullptr;        // and whose it is
    bool rx_spill = false;              // or someone else's, too big for the buffer: dropped
    volatile bool rx_starved = false;   // the task left data in the module for want of a buffer
    bool tx_retry = false;              // a send found the module busy, task only
    TickType_t rested_at = 0;           // when the task last slept, task only
    WifiPeer* volatile tx_peers[WIFI_TX_PEERS] = {};   // the clients written to so far
};

#endif
