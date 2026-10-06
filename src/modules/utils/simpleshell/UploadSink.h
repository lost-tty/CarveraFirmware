#ifndef UPLOADSINK_H
#define UPLOADSINK_H

#include "StreamOutput.h"
#include "libs/SharedBuffer.h"
#include "md5.h"

#include <cstdint>
#include <string>

// Takes an upload's frames as they arrive, in the wifi task or, on serial, in the main loop.
// Writes the file with write(), not stdio: stdio has no locks here.
class UploadSink : public RxSink {
public:
    static const uint16_t SMALL_MAX = 128;   // largest other frame kept, payload bytes

    void start(int fd, SharedBuffer* gather);
    // before asking for the first data frame: count frames of up to max payload bytes
    void expect_data(uint32_t count, uint16_t max);
    // a frame cut short: the main loop is asking again
    void resync() { resync_due = true; }

    void take(const uint8_t* p, size_t n) override;
    size_t reply(uint8_t* out, size_t room) override;
    void settle() override;

    // the main loop's side
    bool next_frame(uint8_t& type, uint8_t* payload, uint16_t& len);
    uint32_t written() const { return good_seq; }   // the last data frame in the file
    bool failed() const { return write_failed; }
    uint32_t size() const { return file_size; }
    std::string md5_hex();

private:
    enum State : uint8_t { HDR0, HDR1, LEN0, LEN1, TYPE, BODY, CRC0, CRC1, FTR0, FTR1 };

    void step(uint8_t c);
    void body(const uint8_t* p, size_t n);
    void end_frame(bool ok);
    void abandon();
    void write(const uint8_t* p, size_t n);

    int fd = -1;
    SharedBuffer* gather = nullptr;

    State state = HDR0;
    uint8_t type = 0;
    uint16_t len = 0, pos = 0, crc = 0, rx_crc = 0;
    uint8_t seq_bytes[4];
    uint32_t seq = 0;
    bool data = false, keep = false;
    bool writing = false;                // the frame waited for, gathered in the shared buffer
    bool busy = false;                   // the frame waited for, with the buffer held elsewhere
    uint16_t data_max = 0;
    uint32_t data_count = 0;

    // written by take(), read by the main loop
    volatile uint32_t next_seq = 0;      // the data frame written next; 0: none yet
    volatile uint32_t good_seq = 0;
    volatile bool resync_due = false;
    volatile bool write_failed = false;
    uint32_t file_size = 0;

    // the feeding task's alone
    uint32_t ask_seq = 0;                // the data frame to ask for; 0: none
    uint32_t held_seq = 0;               // a good frame gathered, not yet in the file; 0: none
    uint32_t held_bytes = 0;
    MD5 md5;

    // other frames, whole: type, length (2), payload; take() appends, the main loop takes.
    // Room for the largest kept (a command line) with its head, and a few small ones besides.
    static const uint16_t RING = 160;
    uint8_t ring[RING];
    volatile uint16_t ring_head = 0, ring_tail = 0;
    uint16_t frame_at = 0;               // where the frame being kept begins
};

#endif
