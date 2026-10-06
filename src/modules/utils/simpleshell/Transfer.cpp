#include "Transfer.h"
#include "FileTransfer.h"
#include "libs/Frame.h"
#include "libs/Kernel.h"
#include "utils.h"
#include "mbed.h"
#include "Program.h"

#include <cstdarg>
#include <cstring>
#include <new>

using Frame::be32;

static inline uint16_t be16(const uint8_t* p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

Transfer::Transfer(FileTransfer& owner, StreamOutput* stream, const std::string& filename)
    : stream(stream), filename(filename), owner(owner), md5()
{
}

// a download's file stays open until its last frame has gone
Transfer::~Transfer()
{
    if (fd != nullptr)
        fclose(fd);
}

void Transfer::use_sink()
{
    new (&sink) UploadSink();
}

void Transfer::use_md5()
{
    new (&md5) MD5();
}

void Transfer::use_frame()
{
    new (&frame) DownloadFrame();
    framing = true;
}

std::string Transfer::md5_path() const
{
    std::string path = change_to_md5_path(filename);
    return is_lz ? FileTransfer::without_lz(path) : path;
}

std::string Transfer::unpacked_name() const
{
    return FileTransfer::without_lz(filename);
}

void Transfer::enter(Phase p)
{
    phase = p;
    retries = 0;
    last_byte = us_ticker_read();
}

void Transfer::send_seq(uint8_t type, uint32_t seq)
{
    uint8_t p[4];
    Frame::put_be32(p, seq);
    stream->send(type, p, sizeof(p));
}

void Transfer::service()
{
    switch (phase) {
        case UP_MD5:
        case UP_VIEW:
        case UP_DATA:
            up_poll();
            break;

        case UP_UNPACK:
            unpack_step();
            break;

        case DOWN_HASH:
            hash_step();
            break;

        case DOWN_WAIT:
            if (frame.busy) {
                check_deadline();
                break;
            }
            if (frame.failed()) {
                cancel("Error: file read error\r\n");
                break;
            }
            down_more();
            check_deadline();
            break;

        case IDLE:
            break;
    }
}

void Transfer::check_deadline()
{
    if (us_ticker_read() - last_byte < IDLE_TIMEOUT_MS * 1000)
        return;

    if (++retries > MAX_RETRIES) {
        cancel(is_download ? "Error: download timed out\r\n" : "Error: upload timed out\r\n");
        return;
    }

    last_byte = us_ticker_read();
    // the client drives a download: it is only late, not lost
    if (is_download)
        return;

    // drop a frame cut short
    sink.resync();
    switch (phase) {
        case UP_MD5:  stream->send(Frame::FILE_MD5, "ok\r\n", 4); break;
        case UP_VIEW: stream->send(Frame::FILE_VIEW, "ok\r\n", 4); break;
        case UP_DATA: send_seq(Frame::FILE_DATA, seq); break;
        default: break;
    }
}

// ---- upload

void Transfer::start_upload()
{
    std::string lz_path = change_to_lz_path(filename);
    check_and_make_path(change_to_md5_path(filename));
    check_and_make_path(lz_path);

    // A .lz lands in the .lz shadow directory and is unpacked to <file>.part afterwards;
    // anything else lands in <file>.part. A failed upload leaves the real file as it was.
    is_lz = filename.find(FileTransfer::LZ_SUFFIX) != std::string::npos;
    want_md5_file = filename.find("firmware.bin") == std::string::npos;
    datafile = is_lz ? lz_path.substr(0, lz_path.rfind(FileTransfer::LZ_SUFFIX))
                     : filename + PART_SUFFIX;

    fd = fopen(datafile.c_str(), "wb");
    if (fd == nullptr) {
        cancel("Error: failed to open file [%s]!\r\n", datafile.substr(0, 30).c_str());
        return;
    }

    use_sink();
    sink.start(fileno(fd), &owner.shared);
    // the upload command's own text first, then the client's bytes go to the sink
    StreamOutput::flush_gathered();
    if (!stream->attach_sink(&sink)) {
        cancel("Error: no upload over this connection\r\n");
        return;
    }

    // the client asks for the md5 first: this phase waits for it and prompts it on a timeout
    enter(UP_MD5);
}

// What the sink reports: other frames in order, then how far the file is; the sink asks for
// the data frames itself.
void Transfer::up_poll()
{
    uint8_t type;
    uint8_t frame[UploadSink::SMALL_MAX];
    uint16_t len;
    while (sink.next_frame(type, frame, len)) {
        last_byte = us_ticker_read();
        up_frame(type, frame, len);
        if (phase != UP_MD5 && phase != UP_VIEW && phase != UP_DATA)
            return;
    }
    if (sink.failed()) {
        cancel("Error: file write error\r\n");
        return;
    }
    uint32_t good = sink.written();
    if (good != seen_good) {
        seen_good = good;
        last_byte = us_ticker_read();
        retries = 0;
        if (good >= total_packets) {
            last_packet();
            return;
        }
        seq = good + 1;
    }
    check_deadline();
}

void Transfer::up_frame(uint8_t type, const uint8_t* p, uint16_t plen)
{
    switch (type) {
        case Frame::FILE_CAN:
            stream->printf("Info: upload canceled by client\r\n");
            finish(false);
            break;

        case Frame::FILE_MD5:
            if (phase == UP_MD5 && plen >= 32) {
                memcpy(client_md5, p, 32);
                client_md5[32] = 0;
                enter(UP_VIEW);
                stream->send(Frame::FILE_VIEW, "ok\r\n", 4);
            }
            // anything else is a stray or short frame: silence, and the deadline re-asks
            break;

        case Frame::FILE_VIEW:
            if (phase == UP_VIEW && plen >= 6) {
                total_packets = be32(p);
                packet_size = be16(p + 4);
                if (total_packets == 0 || packet_size == 0
                    || packet_size > owner.shared.size) {
                    cancel("Error: bad file view\r\n");
                    return;
                }
                enter(UP_DATA);
                seq = 1;
                sink.expect_data(total_packets, packet_size);
            }
            if (phase == UP_DATA)
                send_seq(Frame::FILE_DATA, seq);
            break;

        default:
            // the client's console, multiplexed with its upload: keys, command lines
            if (type < Frame::FILE_MD5 || type > Frame::FILE_RETRY)
                stream->console_frame(type, p, plen);
            break;
    }
}

void Transfer::last_packet()
{
    // the data is in: nothing more reaches the sink, and what it wrote is the file
    stream->detach_sink();
    file_size = sink.size();
    // a .lz is hashed as it unpacks
    if (!is_lz)
        strcpy(file_md5, sink.md5_hex().c_str());

    fclose(fd);
    fd = nullptr;
    use_md5();

    if (is_lz) {
        stream->send(Frame::FILE_END, "ok\r\n", 4);
        enter(UP_UNPACK);
        return;
    }
    if (strcmp(client_md5, file_md5) != 0) {
        cancel("Error: MD5 verification failed\r\n");
        return;
    }
    stream->send(Frame::FILE_END, "ok\r\n", 4);
    finish(true);
}

// Opens the .lz and the unpacked <file>.part; false: it failed, and the upload is finished.
bool Transfer::start_unpack()
{
    packed_size = file_size;
    if (packed_size < BLOCK_HEADER_SIZE + 2) {
        stream->printf("Error: decompression failed\r\n");
        finish(false, true);
        return false;
    }
    f_in = fopen(datafile.c_str(), "rb");
    f_out = fopen((unpacked_name() + PART_SUFFIX).c_str(), "wb");
    if (f_in == nullptr || f_out == nullptr) {
        stream->printf("Error: Failed to open files for decompression!\r\n");
        unpack_finish(false);
        return false;
    }
    memset(&qlz, 0x00, sizeof(qlz));
    unpack_off = 0;
    block_num = 0;
    sum16 = 0;
    return true;
}

void Transfer::unpack_step()
{
    // xbuff is held for one block, so an upload's frame can take it in between
    if (!owner.shared.take(this))
        return;

    if (f_in == nullptr && !start_unpack())
        return;

    uint8_t hdr[BLOCK_HEADER_SIZE];
    uint8_t tail[2];
    if (unpack_off >= packed_size - 2) {
        if (fread(tail, 1, 2, f_in) != 2 || sum16 != ((tail[0] << 8) + tail[1])) {
            unpack_finish(false);
            return;
        }
        strcpy(file_md5, md5.finalize().hexdigest().c_str());
        unpack_finish(true);
        return;
    }

    if (fread(hdr, 1, BLOCK_HEADER_SIZE, f_in) != BLOCK_HEADER_SIZE) {
        unpack_finish(false);
        return;
    }
    uint32_t block_size = be32(hdr);
    if (!block_size || block_size > FileTransfer::XBUFF_SIZE
        || fread(owner.xbuff, 1, block_size, f_in) != block_size) {
        unpack_finish(false);
        return;
    }
    // qlz_decompress reads and writes what the block's own header claims
    if (qlz_size_compressed((const char*)owner.xbuff) > block_size
        || qlz_size_decompressed((const char*)owner.xbuff) > DCOMPRESS_BUFFER_SIZE) {
        unpack_finish(false);
        return;
    }

    uint32_t out = qlz_decompress((const char*)owner.xbuff, owner.lzbuff, &qlz);
    if (!out) {
        unpack_finish(false);
        return;
    }
    for (uint32_t j = 0; j < out; j++)
        sum16 += owner.lzbuff[j];

    md5.update(owner.lzbuff, out);
    if (fwrite(owner.lzbuff, 1, out, f_out) != out) {
        unpack_finish(false);
        return;
    }

    block_num++;
    stream->printf("#Info: decompart = %lu\r\n", (unsigned long)block_num);
    unpack_off += BLOCK_HEADER_SIZE + block_size;
    owner.shared.give(this);
}

void Transfer::unpack_finish(bool ok)
{
    if (f_in != nullptr) {
        fclose(f_in);
        f_in = nullptr;
    }
    if (f_out != nullptr) {
        fclose(f_out);
        f_out = nullptr;
    }

    std::string dest = unpacked_name();
    std::string part = dest + PART_SUFFIX;
    if (!ok) {
        stream->printf("Error: decompression failed\r\n");
        remove(part.c_str());
        finish(false, true);
        return;
    }
    if (strcmp(client_md5, file_md5) != 0) {
        stream->printf("Error: MD5 verification failed\r\n");
        remove(part.c_str());
        finish(false, true);
        return;
    }
    remove(dest.c_str());
    if (rename(part.c_str(), dest.c_str()) != 0) {
        stream->printf("Error: could not rename %s\r\n", part.c_str());
        remove(part.c_str());
        finish(false, true);
        return;
    }
    finish(true);
}

// ---- download

void Transfer::start_download()
{
    // no transfer mode: the requests come as frames through the console (take_frame)
    is_download = true;
    use_md5();

    FILE* f = fopen(change_to_md5_path(filename).c_str(), "rb");
    if (f != nullptr) {
        fgets(file_md5, sizeof(file_md5), f);
        fclose(f);
        open_send_file();
        return;
    }

    fd = fopen(filename.c_str(), "rb");
    if (fd == nullptr) {
        cancel("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        return;
    }
    setvbuf(fd, nullptr, _IONBF, 0);
    enter(DOWN_HASH);
}

void Transfer::take_frame(uint8_t type, const uint8_t* p, uint16_t len)
{
    last_byte = us_ticker_read();
    if (type == Frame::FILE_CAN) {
        stream->printf("Info: download canceled by client\r\n");
        finish(false);
    } else if (phase == DOWN_WAIT) {
        down_frame(type, p, len);
    }
}

void Transfer::hash_step()
{
    int n = owner.hash_piece(fd, md5, this);
    if (n > 0)
        return;

    if (n < 0) {
        cancel("Error: file read error\r\n");
        return;
    }

    strcpy(file_md5, md5.finalize().hexdigest().c_str());
    fclose(fd);
    fd = nullptr;
    open_send_file();
}

void Transfer::open_send_file()
{
    fd = fopen(change_to_lz_path(filename).c_str(), "rb");
    if (fd == nullptr)
        fd = fopen(filename.c_str(), "rb");

    if (fd == nullptr) {
        cancel("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        return;
    }

    setvbuf(fd, nullptr, _IONBF, 0);
    fseek(fd, 0, SEEK_END);
    file_size = (uint32_t)ftell(fd);
    rewind(fd);
    total_packets = (file_size + DOWNLOAD_CHUNK - 1) / DOWNLOAD_CHUNK;

    use_frame();
    down_md5();
    enter(DOWN_WAIT);
}

void Transfer::down_frame(uint8_t type, const uint8_t* p, uint16_t plen)
{
    // any complete frame is proof the client is there
    retries = 0;

    switch (type) {
        case Frame::FILE_MD5:
            down_md5();
            break;

        case Frame::FILE_VIEW:
            down_view();
            break;

        case Frame::FILE_DATA: {
            if (plen < 4)
                break;

            uint32_t rx_seq = be32(p);
            if (rx_seq < 1 || rx_seq > total_packets)
                break;

            // a request is the client's word that it has all before it; the same one again, that
            // a frame came broken, so they go again from there
            if (rx_seq > acked) {
                acked = rx_seq;
                resent_from = 0;
                if (next_send < rx_seq)
                    next_send = rx_seq;
            } else if (rx_seq != resent_from) {
                resent_from = rx_seq;
                next_send = rx_seq;
            }
            down_more();
            break;
        }

        case Frame::FILE_RETRY:
            if (last_sent == SENT_MD5)
                down_md5();
            else if (last_sent == SENT_VIEW)
                down_view();
            else if (last_sent == SENT_DATA) {
                next_send = last_seq;
                down_more();
            }

            break;

        case Frame::FILE_END:
            stream->send(Frame::FILE_END, "ok\r\n", 4);
            finish(true);
            break;

        default:
            break;
    }
}

void Transfer::down_md5()
{
    stream->send(Frame::FILE_MD5, file_md5, 32);
    last_sent = SENT_MD5;
}

void Transfer::down_view()
{
    uint8_t v[6] = { 0, 0, 0, 0, (uint8_t)(DOWNLOAD_CHUNK >> 8), (uint8_t)DOWNLOAD_CHUNK };
    Frame::put_be32(v, total_packets);
    stream->send(Frame::FILE_VIEW, v, sizeof(v));
    last_sent = SENT_VIEW;
}

// the next frame the window allows, once the one before has gone
void Transfer::down_more()
{
    if (frame.busy || next_send > total_packets || next_send >= acked + DOWNLOAD_WINDOW)
        return;

    down_data(next_send++);
}

void Transfer::down_data(uint32_t rx_seq)
{
    uint32_t off = (rx_seq - 1) * DOWNLOAD_CHUNK;
    uint32_t n = file_size - off < DOWNLOAD_CHUNK ? file_size - off : DOWNLOAD_CHUNK;
    frame.start(fileno(fd), rx_seq, (long)off, (uint16_t)n);
    stream->puts_source(&frame);
    last_sent = SENT_DATA;
    last_seq = rx_seq;
}

// ---- the end, either way

void Transfer::cancel(const char* fmt, ...)
{
    stream->send(Frame::FILE_CAN, "ok\r\n", 4);
    va_list args;
    va_start(args, fmt);
    stream->vprintf(fmt, args);
    va_end(args);
    finish(false);
}

void Transfer::finish(bool ok, bool keep_data, bool quiet)
{
    // a frame cut short may still hold xbuff
    if (!is_download) {
        stream->detach_sink();
        owner.shared.give(&sink);
    }

    if (fd != nullptr && !is_download) {
        fclose(fd);
        fd = nullptr;
    }
    if (f_in != nullptr) {
        fclose(f_in);
        f_in = nullptr;
    }
    if (f_out != nullptr) {
        fclose(f_out);
        f_out = nullptr;
        remove((unpacked_name() + PART_SUFFIX).c_str());
    }
    if (!ok && !is_download && !keep_data)
        remove(datafile.c_str());

    if (ok && !is_download && !is_lz) {
        remove(filename.c_str());
        ok = rename(datafile.c_str(), filename.c_str()) == 0;
        if (!ok && !quiet)
            stream->printf("Error: could not rename %s\r\n", datafile.c_str());

        if (!ok)
            remove(datafile.c_str());
    }

    // the md5 a download sends without hashing again; a failed upload leaves the old file,
    // and with it its md5
    if (ok && !is_download && want_md5_file) {
        FILE* f = fopen(md5_path().c_str(), "wb");
        if (f != nullptr) {
            fwrite(file_md5, 1, strlen(file_md5), f);
            fclose(f);
        }
    }

    if (!quiet) {
        if (is_download) {
            stream->printf(ok ? "Info: Download success: %s.\r\n"
                              : "Download failed for file: %s.\r\n", filename.c_str());
        } else if (ok) {
            program.macros().file_changed(filename.c_str());
            stream->printf("Info: upload success: %s.\r\n", filename.c_str());
        } else {
            stream->printf("Upload failed for file: %s.\r\n", filename.c_str());
        }
    }
    phase = IDLE;
}
