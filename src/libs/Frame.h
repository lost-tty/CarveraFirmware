// Makera frame: 86 68 | len:2 | type:1 | payload | crc16:2 | 55 AA   (big endian,
// len = payload + 3, CRC16-CCITT poly 0x1021 init 0 over len, type and payload)
#ifndef FRAME_H
#define FRAME_H

#include <cstdint>
#include <cstddef>

namespace Frame {

enum Type : uint8_t {
    // firmware -> client
    STATUS      = 0x81,  // reply to '?'
    DIAG        = 0x82,  // reply to 'diagnose'
    LOAD_INFO   = 0x83,  // chunk of a list reply (ls, wlan)
    LOAD_FINISH = 0x84,  // list reply complete
    LOAD_ERROR  = 0x85,  // list reply failed
    INFO        = 0x90,  // any other text, including "ok"
    // client -> firmware
    CTRL_SINGLE = 0xA1,  // one realtime byte: ? ! ~ ^X
    CTRL_MULTI  = 0xA2,  // one command line
    FILE_START  = 0xB0,  // the "upload <file>" command line
    // file transfer, both directions
    FILE_MD5    = 0xB1,
    FILE_VIEW   = 0xB2,
    FILE_DATA   = 0xB3,
    FILE_END    = 0xB4,
    FILE_CAN    = 0xB5,
    FILE_RETRY  = 0xB6,
};

static const uint16_t HEADER   = 0x8668;
static const uint16_t FOOTER   = 0x55AA;
static const size_t   OVERHEAD = 9;      // header 2 + len 2 + type 1 + crc 2 + footer 2

uint16_t crc16(uint16_t crc, const uint8_t *data, size_t len);

// out must hold len + OVERHEAD bytes; returns bytes written
size_t encode(uint8_t type, const void *payload, size_t len, uint8_t *out);

class Decoder {
public:
    Decoder(uint8_t *buf, size_t capacity) : buf(buf), cap(capacity) { reset(); }

    // true when a complete valid frame is available via type()/payload()/length()
    bool feed(uint8_t c);
    void reset();

    uint8_t        type()    const { return type_; }
    const uint8_t *payload() const { return buf; }
    uint16_t       length()  const { return len_; }

private:
    enum State : uint8_t { HDR0, HDR1, LEN0, LEN1, TYPE, PAYLOAD, CRC0, CRC1, FTR0, FTR1 };

    uint8_t *buf;
    size_t   cap;
    State    state;
    uint16_t len_;      // payload length
    uint16_t pos;
    uint16_t crc;
    uint16_t rx_crc;
    uint8_t  type_;
};

} // namespace Frame

#endif
