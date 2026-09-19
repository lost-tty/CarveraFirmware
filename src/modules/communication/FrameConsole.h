#pragma once

#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"
#include "libs/Frame.h"

#include <string>

// The client side of the Makera frame protocol: a transport fills rx with bytes as they arrive
// and calls decode() from the main loop, control frames are answered and command lines queued.
class FrameConsole : public StreamOutput {
public:
    static const int RX_LINE_BUF = 256;      // power of two, RingBuffer requires it

    void set_transferring(bool f) override { transferring= f; if(!f) decoder.reset(); }
    bool is_transferring() const override { return transferring; }

protected:
    // one frame per call: its command may start a transfer, and the bytes behind it are then payload
    template<int N> void decode(RingBuffer<char, N> &rx) {
        if (transferring) return;
        while (rx.size() > 0) {
            char c;
            rx.pop_front(c);
            if (decoder.feed(c)) {
                on_frame();
                return;
            }
        }
    }

    bool next_line(std::string &line);
    void on_frame();

    bool transferring= false;
    RingBuffer<char, RX_LINE_BUF> buffer;    // decoded command lines, '\n' terminated
    static const size_t RX_FRAME_MAX = 256;  // largest accepted command frame payload; longer frames are dropped
    uint8_t rx_frame[RX_FRAME_MAX];
    Frame::Decoder decoder{rx_frame, sizeof(rx_frame)};
};
