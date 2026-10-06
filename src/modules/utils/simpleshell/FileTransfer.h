// File transfer over Makera frames, protocol as in upstream Player::upload_command / download_command
#ifndef FILETRANSFER_H
#define FILETRANSFER_H

#include <string>
#include <cstdint>
#include "StreamOutput.h"
#include "quicklz.h"
#include "libs/SharedBuffer.h"
#include "libs/SoftTimer.h"
#include "md5.h"

#include <cstdio>

class Transfer;

// The file transfers under way, at most one per client.
class FileTransfer {
public:
    void begin(const std::string& filename, StreamOutput* stream, bool upload);

    void service();

    static constexpr const char *LZ_SUFFIX = ".lz";
    static std::string without_lz(const std::string& name)
    {
        return name.substr(0, name.find(LZ_SUFFIX));
    }

    void cancel_if(StreamOutput* s);
    // a download's request, from its stream's console decoder; false: not ours
    bool take_frame(StreamOutput* s, uint8_t type, const uint8_t* p, uint16_t len);

    static const size_t XBUFF_SIZE = COMPRESS_BUFFER_SIZE + BUFFER_PADDING;
    // holds one unpacked block, or one upload data frame until its CRC is checked; the card
    // then writes the frame in one multi-block write instead of sector by sector
    unsigned char xbuff[XBUFF_SIZE + DCOMPRESS_BUFFER_SIZE];
    unsigned char* const lzbuff = xbuff + XBUFF_SIZE;
    SharedBuffer shared{xbuff, sizeof(xbuff)};

    // The next piece of f into md5: 8 KB through xbuff while no one else holds it, else 512
    // bytes. Bytes hashed, 0 at the end, < 0 on a read error.
    int hash_piece(FILE* f, MD5& md5, const void* who);

private:
    // at most one per console client
    static const int MAX_TRANSFERS = 4;
    Transfer* transfers[MAX_TRANSFERS] = {};

    Transfer* start(const std::string& filename, StreamOutput* stream, bool upload);
    Transfer* of(const StreamOutput* s) const;
    void end(Transfer* t);

    // the main loop's look at the transfers' timeouts, while there are any
    void tick();
    void count_changed();
    SoftTimer ticker{"Transfer", 100, true, this, &FileTransfer::tick};
};

#endif
