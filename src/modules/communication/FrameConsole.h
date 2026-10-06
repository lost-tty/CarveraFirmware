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

    void console_frame(uint8_t type, const uint8_t *p, uint16_t len) override;

    // returns the bytes taken: all of them, unless a frame started a transfer or asked for one
    uint16_t feed(const uint8_t *p, uint16_t len);
    void pump();

    // what was queued, and a frame half decoded
    void drop_queued() { buffer.tail = buffer.head; decoder.reset(); }

protected:
    bool next_line(std::string &line);
    void queue_frame(uint8_t type, const uint8_t *p, uint16_t len);
    void on_frame();
    void handle_key(uint8_t c);

    RingBuffer<char, RX_LINE_BUF> buffer;    // decoded command lines, '\n' terminated
    static const size_t RX_FRAME_MAX = 128;  // largest accepted command frame payload; longer frames are dropped
    uint8_t rx_frame[RX_FRAME_MAX];
    Frame::Decoder decoder{rx_frame, sizeof(rx_frame)};
};
