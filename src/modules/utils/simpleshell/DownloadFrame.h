#ifndef DOWNLOADFRAME_H
#define DOWNLOADFRAME_H

#include "StreamOutput.h"

#include <cstdint>

// Reads below stdio: newlib's stdio has no locks here, and an unbuffered fread flushes every
// open output stream. A failed read breaks the CRC.
class DownloadFrame : public TxSource {
public:
    static const size_t HEAD = 9;        // header 2, len 2, type 1, seq 4
    static const size_t TAIL = 4;        // crc 2, footer 2

    void start(int fd, uint32_t seq, long off, uint16_t n);
    bool failed() const { return read_failed; }

    size_t size() const override { return HEAD + n + TAIL; }
    void read(size_t at, uint8_t* p, size_t len) override;

private:
    uint8_t byte_at(size_t at) const;
    void read_chunk(size_t from, uint8_t* p, size_t k);

    int fd = -1;
    uint32_t seq = 0;
    long off = 0;
    uint16_t n = 0;
    uint16_t crc = 0;
    size_t summed = 0;                   // the frame's bytes in crc so far
    long file_at = -1;                   // where fd stands, -1: unknown
    volatile bool read_failed = false;
};

#endif
