// File transfer over Makera frames, protocol as in upstream Player::upload_command / download_command
#ifndef FILETRANSFER_H
#define FILETRANSFER_H

#include <string>
#include <cstdint>
#include <cstdio>
#include "StreamOutput.h"
#include "libs/Frame.h"
#include "quicklz.h"
#include "md5.h"

class FileTransfer {
public:
    void upload(const std::string& filename, StreamOutput* stream);
    void download(const std::string& filename, StreamOutput* stream);

    void service();

    static constexpr const char *LZ_SUFFIX = ".lz";

    void cancel_if(StreamOutput* s);


private:
    enum Phase : uint8_t { IDLE, UP_MD5, UP_VIEW, UP_DATA, UP_UNPACK, DOWN_HASH, DOWN_WAIT };

    void start_upload(const std::string& filename, StreamOutput* stream);
    void start_download(const std::string& filename, StreamOutput* stream);

    void enter(Phase p);
    void check_deadline();
    void up_frame(const Frame::Decoder& dec);
    void down_frame(const Frame::Decoder& dec);
    void last_packet();
    void hash_step();
    void open_send_file();
    void unpack_step();
    void unpack_finish(bool ok);
    void finish(bool ok, bool keep_data = false, bool quiet = false);

    void send_seq(uint8_t type, uint32_t seq);

    StreamOutput *active_stream = nullptr;

    static const size_t XBUFF_SIZE = COMPRESS_BUFFER_SIZE + BUFFER_PADDING;
    unsigned char xbuff[XBUFF_SIZE + DCOMPRESS_BUFFER_SIZE];
    unsigned char* const lzbuff = xbuff + XBUFF_SIZE;
    Frame::Decoder up_decoder{xbuff, sizeof(xbuff)};

    uint8_t ctrl[64];
    Frame::Decoder down_decoder{ctrl, sizeof(ctrl)};

    Phase phase = IDLE;
    bool is_download = false;

    FILE* fd = nullptr;
    FILE* f_md5 = nullptr;
    MD5 md5;
    std::string filename, datafile, path_md5, path_lz, dest;
    bool is_lz = false, hash_on_wire = false, want_md5_file = false;
    std::string received_md5, computed_md5;
    uint32_t total_packets = 0, seq = 1, file_size = 0;
    uint16_t packet_size = 0;
    char md5_str[33] = { 0 };
    size_t last_len = 0;

    int retries = 0;
    uint32_t last_byte = 0;

    FILE* f_in = nullptr;
    FILE* f_out = nullptr;
    qlz_state_decompress qlz;
    uint32_t unpack_off = 0, block_size = 0, packed_size = 0, block_num = 0;
    uint16_t sum16 = 0;

    static constexpr const char *PART_SUFFIX = ".part";

    static const uint32_t IDLE_TIMEOUT_MS = 500;    // a phase's silence before asking again
    static const int      MAX_RETRIES     = 20;     // asks before giving up (10 s idle)
    static const size_t   DOWNLOAD_CHUNK  = 4000;   // data bytes per FILE_DATA frame we send
};

#endif
