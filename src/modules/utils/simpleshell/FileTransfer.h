// File transfer over Makera frames, protocol as in upstream Player::upload_command / download_command
#ifndef FILETRANSFER_H
#define FILETRANSFER_H

#include <string>
#include <cstdint>
#include <cstdio>
#include "StreamOutput.h"
#include "quicklz.h"

class FileTransfer {
public:
    bool upload(const std::string& filename, StreamOutput* stream);
    bool download(const std::string& filename, StreamOutput* stream);

private:
    int  in(StreamOutput* stream, uint32_t timeout_ms);
    void flush_input(StreamOutput* stream);

    // returns the frame type or -1 on timeout; crc is the running CRC over len and type
    int  read_header(StreamOutput* stream, uint32_t timeout_ms, uint16_t& plen, uint16_t& crc);
    bool read_payload(StreamOutput* stream, uint8_t* dst, uint16_t plen, uint16_t crc);
    void skip_payload(StreamOutput* stream, uint16_t plen);
    bool read_trailer(StreamOutput* stream, uint16_t crc);

    void send_seq(StreamOutput* stream, uint8_t type, uint32_t seq);
    void set_serial_rx_irq(bool enable);
    bool decompress(const std::string& sfilename, const std::string& dfilename, uint32_t sfilesize, StreamOutput* stream);

    // whole incoming packet during a transfer; compressed block + output during decompression
    static const size_t XBUFF_SIZE = COMPRESS_BUFFER_SIZE + BUFFER_PADDING;
    unsigned char xbuff[XBUFF_SIZE + DCOMPRESS_BUFFER_SIZE];
    unsigned char* const lzbuff = xbuff + XBUFF_SIZE;

    char* pend = nullptr;
    int   pend_len = 0;

    static const uint32_t BYTE_TIMEOUT_MS = 200;    // between bytes inside one frame
    static const uint32_t IDLE_TIMEOUT_MS = 500;    // waiting for the next frame before repeating a request
    static const int      MAX_RETRIES     = 60;     // repeated requests before giving up (30 s idle)
    static const size_t   DOWNLOAD_CHUNK  = 4000;   // data bytes per FILE_DATA frame we send
};

#endif
