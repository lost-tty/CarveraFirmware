#include "UploadSink.h"

#include "libs/Frame.h"
#include "libs/MainWake.h"

#include <algorithm>
#include <cstring>
#include <unistd.h>

namespace {
    // the ring's contents and its indices pass between the tasks: neither side's accesses to
    // the one may move past its write of the other
    inline void ordered()
    {
        __asm volatile("" ::: "memory");
    }
}

void UploadSink::start(int fd, SharedBuffer* gather)
{
    this->fd = fd;
    this->gather = gather;
    writing = busy = false;
    state = HDR0;
    data_max = 0;
    next_seq = good_seq = 0;
    ask_seq = held_seq = held_bytes = 0;
    md5 = MD5();
    resync_due = write_failed = false;
    file_size = 0;
    ring_head = ring_tail = 0;
}

void UploadSink::expect_data(uint32_t count, uint16_t max)
{
    data_count = count;
    data_max = max;
    ordered();
    next_seq = 1;
}

void UploadSink::take(const uint8_t* p, size_t n)
{
    if (resync_due) {
        abandon();
        resync_due = false;
    }
    while (n > 0) {
        if (state == BODY) {
            size_t run = std::min<size_t>(n, len - pos);
            body(p, run);
            p += run;
            n -= run;
            continue;
        }
        step(*p++);
        n--;
    }
}

// The frame's head and tail, a byte at a time, as Frame::Decoder reads them.
void UploadSink::step(uint8_t c)
{
    switch (state) {
    case HDR0:
        if (c == (Frame::HEADER >> 8))
            state = HDR1;
        break;

    case HDR1:
        if (c == (Frame::HEADER & 0xFF))
            state = LEN0;
        else if (c != (Frame::HEADER >> 8))
            state = HDR0;
        break;

    case LEN0:
        len = (uint16_t)c << 8;
        crc = Frame::crc16(0, &c, 1);
        state = LEN1;
        break;

    case LEN1:
        len |= c;
        crc = Frame::crc16(crc, &c, 1);
        if (len < 3) {
            state = HDR0;
            break;
        }
        len -= 3;
        state = TYPE;
        break;

    case TYPE: {
        type = c;
        crc = Frame::crc16(crc, &c, 1);
        pos = 0;
        data = type == Frame::FILE_DATA;
        // a data frame longer than asked for is not written; another frame is kept only if
        // it fits whole into the ring
        writing = busy = false;
        keep = false;
        if (!data && len <= SMALL_MAX) {
            uint16_t used = (ring_tail + RING - ring_head) % RING;
            keep = RING - 1 - used >= 3 + len;
            frame_at = ring_tail;
        }
        state = len ? BODY : CRC0;
        break;
    }

    case CRC0:
        rx_crc = (uint16_t)c << 8;
        state = CRC1;
        break;

    case CRC1:
        rx_crc |= c;
        state = FTR0;
        break;

    case FTR0:
        if (c == (Frame::FOOTER >> 8)) {
            state = FTR1;
        } else {
            end_frame(false);
        }
        break;

    case FTR1:
        end_frame(c == (Frame::FOOTER & 0xFF) && rx_crc == crc);
        break;

    case BODY:
        break;
    }
}

void UploadSink::body(const uint8_t* p, size_t n)
{
    crc = Frame::crc16(crc, p, n);
    if (data) {
        // the sequence number first: only the frame waited for is written
        while (pos < 4 && n > 0) {
            seq_bytes[pos++] = *p++;
            n--;
            if (pos == 4) {
                seq = Frame::be32(seq_bytes);
                bool wanted = next_seq != 0 && seq == next_seq && len - 4u <= data_max
                              && !write_failed;
                // a client sending ahead: the frame before goes to the card first
                if (wanted)
                    settle();

                writing = wanted && gather->take(this);
                busy = wanted && !writing;
            }
        }
        if (writing && n > 0)
            memcpy(gather->data + pos - 4, p, n);
    } else if (keep) {
        for (size_t i = 0; i < n; i++)
            ring[(frame_at + 3 + pos + i) % RING] = p[i];
    }
    pos += n;
    if (pos == len)
        state = CRC0;
}

void UploadSink::end_frame(bool ok)
{
    state = HDR0;
    if (writing) {
        writing = false;
        if (ok) {
            // settle() writes it, after the reply went out
            held_seq = seq;
            held_bytes = len - 4;
            next_seq = seq + 1;
            ask_seq = seq < data_count ? seq + 1 : 0;
            return;
        }
        gather->give(this);
        ask_seq = seq;
        return;
    }
    if (busy) {
        busy = false;
        ask_seq = seq;
        return;
    }
    if (keep && ok) {
        ring[frame_at % RING] = type;
        ring[(frame_at + 1) % RING] = len >> 8;
        ring[(frame_at + 2) % RING] = len & 0xFF;
        ordered();
        ring_tail = (frame_at + 3 + len) % RING;
        wake_main();
    }
}

// A frame cut short: dropped as if its CRC had failed, but not asked for again.
void UploadSink::abandon()
{
    if (writing) {
        writing = false;
        gather->give(this);
    }
    busy = false;
    state = HDR0;
}

void UploadSink::write(const uint8_t* p, size_t n)
{
    if (::write(fd, p, n) != (ssize_t)n)
        write_failed = true;
}

bool UploadSink::next_frame(uint8_t& type, uint8_t* payload, uint16_t& len)
{
    uint16_t head = ring_head;
    if (head == ring_tail)
        return false;

    ordered();
    type = ring[head];
    len = (uint16_t)ring[(head + 1) % RING] << 8 | ring[(head + 2) % RING];
    for (uint16_t i = 0; i < len; i++)
        payload[i] = ring[(head + 3 + i) % RING];

    ordered();
    ring_head = (head + 3 + len) % RING;
    return true;
}

size_t UploadSink::reply(uint8_t* out, size_t room)
{
    if (ask_seq == 0 || room < Frame::OVERHEAD + 4)
        return 0;

    uint8_t p[4];
    Frame::put_be32(p, ask_seq);
    ask_seq = 0;
    return Frame::encode(Frame::FILE_DATA, p, sizeof(p), out);
}

void UploadSink::settle()
{
    if (held_seq == 0)
        return;

    md5.update(gather->data, held_bytes);
    write(gather->data, held_bytes);
    gather->give(this);
    file_size += held_bytes;
    ordered();
    good_seq = held_seq;
    held_seq = 0;
    wake_main();
}

std::string UploadSink::md5_hex()
{
    return md5.finalize().hexdigest();
}
