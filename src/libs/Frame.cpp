#include "Frame.h"

#include <cstring>

namespace Frame {

uint16_t crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

size_t encode(uint8_t type, const void *payload, size_t len, uint8_t *out)
{
    uint16_t flen = len + 3;
    out[0] = HEADER >> 8;
    out[1] = HEADER & 0xFF;
    out[2] = flen >> 8;
    out[3] = flen & 0xFF;
    out[4] = type;
    if (len) memcpy(out + 5, payload, len);
    uint16_t crc = crc16(0, out + 2, flen);
    out[5 + len] = crc >> 8;
    out[6 + len] = crc & 0xFF;
    out[7 + len] = FOOTER >> 8;
    out[8 + len] = FOOTER & 0xFF;
    return len + OVERHEAD;
}

void Decoder::reset()
{
    state = HDR0;
    len_ = pos = crc = rx_crc = 0;
    type_ = 0;
}

bool Decoder::feed(uint8_t c)
{
    switch (state) {
    case HDR0:
        if (c == (HEADER >> 8)) state = HDR1;
        return false;

    case HDR1:
        if (c == (HEADER & 0xFF)) {
            state = LEN0;
        } else {
            state = (c == (HEADER >> 8)) ? HDR1 : HDR0;
        }
        return false;

    case LEN0:
        len_ = (uint16_t)c << 8;
        crc = crc16(0, &c, 1);
        state = LEN1;
        return false;

    case LEN1:
        len_ |= c;
        crc = crc16(crc, &c, 1);
        if (len_ < 3 || (size_t)(len_ - 3) > cap) {
            reset();
            return false;
        }
        len_ -= 3;
        pos = 0;
        state = TYPE;
        return false;

    case TYPE:
        type_ = c;
        crc = crc16(crc, &c, 1);
        state = len_ ? PAYLOAD : CRC0;
        return false;

    case PAYLOAD:
        buf[pos++] = c;
        crc = crc16(crc, &c, 1);
        if (pos == len_) state = CRC0;
        return false;

    case CRC0:
        rx_crc = (uint16_t)c << 8;
        state = CRC1;
        return false;

    case CRC1:
        rx_crc |= c;
        if (rx_crc != crc) {
            reset();
            return false;
        }
        state = FTR0;
        return false;

    case FTR0:
        state = (c == (FOOTER >> 8)) ? FTR1 : HDR0;
        return false;

    case FTR1:
        state = HDR0;
        return c == (FOOTER & 0xFF);
    }
    return false;
}

} // namespace Frame
