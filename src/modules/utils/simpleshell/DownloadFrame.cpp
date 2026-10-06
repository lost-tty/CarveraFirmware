#include "DownloadFrame.h"
#include "libs/Frame.h"

#include <cstring>
#include <unistd.h>

void DownloadFrame::start(int fd, uint32_t seq, long off, uint16_t n)
{
    if (fd != this->fd)
        file_at = -1;

    this->fd = fd;
    this->seq = seq;
    this->off = off;
    this->n = n;
    read_failed = false;
    uint8_t head[HEAD];
    for (size_t i = 0; i < HEAD; i++)
        head[i] = byte_at(i);

    crc = Frame::crc16(0, head + 2, HEAD - 2);
    summed = HEAD;
}

uint8_t DownloadFrame::byte_at(size_t at) const
{
    uint16_t flen = 4 + n + 3;
    switch (at) {
        case 0: return Frame::HEADER >> 8;
        case 1: return Frame::HEADER & 0xFF;
        case 2: return flen >> 8;
        case 3: return flen & 0xFF;
        case 4: return Frame::FILE_DATA;
        case 5: return seq >> 24;
        case 6: return seq >> 16;
        case 7: return seq >> 8;
        case 8: return seq;
    }
    switch (at - HEAD - n) {
        case 0: return crc >> 8;
        case 1: return crc & 0xFF;
        case 2: return Frame::FOOTER >> 8;
        default: return Frame::FOOTER & 0xFF;
    }
}

void DownloadFrame::read(size_t at, uint8_t* p, size_t len)
{
    size_t end = at + len;
    size_t data_end = HEAD + n;
    size_t i = at;
    while (i < end) {
        if (i < HEAD || i >= data_end) {
            p[i - at] = byte_at(i);
            i++;
            continue;
        }
        size_t k = (end < data_end ? end : data_end) - i;
        read_chunk(i - HEAD, p + (i - at), k);
        // a piece asked for again is in the CRC already
        if (i <= summed && summed < i + k) {
            crc = Frame::crc16(crc, p + (summed - at), i + k - summed);
            summed = i + k;
            if (summed == data_end && read_failed)
                crc = ~crc;
        }
        i += k;
    }
}

void DownloadFrame::read_chunk(size_t from, uint8_t* p, size_t k)
{
    long want = off + (long)from;
    if (file_at != want)
        file_at = ::lseek(fd, want, SEEK_SET) == want ? want : -1;

    int got = file_at < 0 ? 0 : ::read(fd, p, k);
    if (got < 0)
        got = 0;

    if ((size_t)got < k) {
        memset(p + got, 0, k - got);
        read_failed = true;
        file_at = -1;
        return;
    }
    file_at += got;
}
