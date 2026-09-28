#pragma once

#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"
#include "libs/Frame.h"

#include <string>

// The client side of the Makera frame protocol: a transport fills rx with bytes as they arrive
// and calls decode() from the main loop, control frames are answered and command lines queued.
class FrameConsole : public StreamOutput {
public:
    static const int RX_LINE_BUF = 128;      // power of two, RingBuffer requires it

    void set_transferring(bool f) override { transferring= f; if(!f) decoder.reset(); }
    bool is_transferring() const override { return transferring; }

    void feed(const uint8_t *p, uint16_t len) { run_bytes(p, len, true); }
    void queue(const uint8_t *p, uint16_t len) { run_bytes(p, len, false); }

    void drop_queued() { buffer.tail = buffer.head; keys.tail = keys.head; }

protected:
    void run_bytes(const uint8_t *p, uint16_t len, bool dispatch);

    bool next_line(std::string &line);
    void queue_frame();
    void on_frame();
    void handle_key(uint8_t c);
    bool act_key();

    bool transferring= false;
    RingBuffer<char, RX_LINE_BUF> buffer;    // decoded command lines, '\n' terminated
    static const int KEY_QUEUE = 8;          // power of two, RingBuffer requires it
    RingBuffer<uint8_t, KEY_QUEUE> keys;     // control bytes that arrived while the pump was busy
    static const size_t RX_FRAME_MAX = 128;  // largest accepted command frame payload; longer frames are dropped
    uint8_t rx_frame[RX_FRAME_MAX];
    Frame::Decoder decoder{rx_frame, sizeof(rx_frame)};
};
