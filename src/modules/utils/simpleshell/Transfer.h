#ifndef TRANSFER_H
#define TRANSFER_H

#include <string>
#include <cstdint>
#include <cstdio>
#include "StreamOutput.h"
#include "quicklz.h"
#include "md5.h"
#include "UploadSink.h"
#include "DownloadFrame.h"

class FileTransfer;

// One upload or download, for one client. An upload's bytes go to its sink as they arrive and
// into <file>.part; once all are in, its MD5 (the sink's, or the file read back) is compared
// with the client's and the file renamed (a .lz is unpacked into <file>.part instead, hashed as
// it goes). A download answers the client's requests a frame at a time.
class Transfer {
public:
    Transfer(FileTransfer& owner, StreamOutput* stream, const std::string& filename);
    ~Transfer();

    void start_upload();
    void start_download();
    void service();
    // a download's request, from the client's console decoder
    void take_frame(uint8_t type, const uint8_t* p, uint16_t len);
    void finish(bool ok, bool keep_data = false, bool quiet = false);
    // FILE_CAN to the client, the error to its console, and finish(false)
    void cancel(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    bool done() const { return phase == IDLE; }
    bool sending() const { return framing && frame.busy; }
    // a step of hashing or unpacking (or its try for xbuff) is due at once
    bool working() const { return phase == DOWN_HASH || phase == UP_UNPACK; }

    StreamOutput* const stream;
    const std::string filename;
    bool is_download = false;

private:
    enum Phase : uint8_t {
        IDLE,
        UP_MD5, UP_VIEW, UP_DATA,        // the client's md5, its file's view, its data
        UP_UNPACK,                       // a .lz unpacked, a block per turn with xbuff
        DOWN_HASH, DOWN_WAIT,            // the file hashed, then the client's requests
    };

    void enter(Phase p);
    void check_deadline();
    void send_seq(uint8_t type, uint32_t seq);

    void up_poll();
    void up_frame(uint8_t type, const uint8_t* p, uint16_t plen);
    void last_packet();
    bool start_unpack();
    void unpack_step();
    void unpack_finish(bool ok);

    void hash_step();
    void open_send_file();
    void down_frame(uint8_t type, const uint8_t* p, uint16_t plen);
    void down_md5();
    void down_view();
    void down_data(uint32_t rx_seq);
    void down_more();

    std::string md5_path() const;
    std::string unpacked_name() const;

    FileTransfer& owner;
    Phase phase = IDLE;

    // The sink while an upload's data comes in, the hash from then on; a download's hash, then
    // its data frame: never two at once, so they share the memory.
    union {
        UploadSink sink;
        MD5 md5;
        DownloadFrame frame;
    };
    void use_sink();
    void use_md5();
    void use_frame();
    bool framing = false;

    std::string datafile;                // where the data lands: <file>.part, or the .lz shadow
    bool is_lz = false, want_md5_file = false;
    char client_md5[33] = {};            // the md5 the client gave
    char file_md5[33] = {};              // the file's: computed, or a download's from .md5
    uint32_t total_packets = 0, seq = 1, file_size = 0, seen_good = 0;
    uint16_t packet_size = 0;

    FILE* fd = nullptr;
    int retries = 0;
    uint32_t last_byte = 0;

    // a download: the frame a retry sends again; the client has every frame before acked, and
    // frames go out ahead of its requests up to DOWNLOAD_WINDOW past it
    enum Sent : uint8_t { SENT_NONE, SENT_MD5, SENT_VIEW, SENT_DATA };
    Sent last_sent = SENT_NONE;
    uint32_t last_seq = 0;
    uint32_t acked = 0, next_send = 1, resent_from = 0;

    // a .lz upload's unpacking
    FILE* f_in = nullptr;
    FILE* f_out = nullptr;
    qlz_state_decompress qlz;
    uint32_t unpack_off = 0, packed_size = 0, block_num = 0;
    uint16_t sum16 = 0;

    static constexpr const char *PART_SUFFIX = ".part";
    static const uint32_t IDLE_TIMEOUT_MS = 500;    // a phase's silence before asking again
    static const int      MAX_RETRIES     = 20;     // asks before giving up (10 s idle)
    static const size_t   DOWNLOAD_CHUNK  = 4000;   // data bytes per FILE_DATA frame we send
    static const uint32_t DOWNLOAD_WINDOW = 16;     // 1: each frame only when asked for
};

#endif
