#include "FileTransfer.h"
#include "libs/Kernel.h"
#include "Conveyor.h"
#include "utils.h"
#include "mbed.h"
#include "Scripts.h"
#include <cstring>

using namespace std;

static inline uint32_t be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline uint16_t be16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | p[1]; }

void FileTransfer::send_seq(uint8_t type, uint32_t seq)
{
    uint8_t p[4] = { (uint8_t)(seq >> 24), (uint8_t)(seq >> 16), (uint8_t)(seq >> 8), (uint8_t)seq };
    active_stream->send(type, p, sizeof(p));
}

void FileTransfer::cancel_if(StreamOutput* s)
{
    // called from Session::bind/release on the main loop, so the step cannot be mid-flight:
    // end it here, before the slot's next client can inherit the failure report
    if (active_stream == s) finish(false, false, true);
}

void FileTransfer::upload(const std::string& filename, StreamOutput* stream)
{
    if (active_stream) {
        stream->printf("error:another transfer is in progress\r\n");
        return;
    }
    start_upload(filename, stream);
}

void FileTransfer::start_upload(const std::string& filename, StreamOutput* stream)
{
    this->filename = filename;
    path_md5 = change_to_md5_path(filename);
    path_lz = change_to_lz_path(filename);
    check_and_make_path(path_md5);
    check_and_make_path(path_lz);

    // .lz uploads land in the .lz shadow directory and are decompressed to filename afterwards
    is_lz = filename.find(".lz") != string::npos;
    datafile = filename;
    if (is_lz) {
        datafile = path_lz.substr(0, path_lz.rfind(".lz"));
        path_md5 = path_md5.substr(0, path_md5.find(".lz"));
    }
    want_md5_file = filename.find("firmware.bin") == string::npos;
    hash_on_wire = !is_lz;

    // a failed transfer leaves the .part behind, not a truncated file under the real name
    if (!is_lz) datafile += PART_SUFFIX;

    active_stream = stream;
    stream->set_transferring(true);
    md5 = MD5();

    fd = fopen(datafile.c_str(), "wb");
    f_md5 = want_md5_file ? fopen(path_md5.c_str(), "wb") : nullptr;
    if (fd == nullptr || (want_md5_file && f_md5 == nullptr)) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        stream->printf("Error: failed to open file [%s]!\r\n",
                       fd == nullptr ? datafile.substr(0, 30).c_str() : path_md5.substr(0, 30).c_str());
        finish(false);
        return;
    }

    // the client asks for the md5 first: this phase waits for it and prompts it on a timeout
    enter(UP_MD5);
}

void FileTransfer::download(const std::string& filename, StreamOutput* stream)
{
    if (active_stream) {
        stream->printf("error:another transfer is in progress\r\n");
        return;
    }
    start_download(filename, stream);
}

void FileTransfer::start_download(const std::string& filename, StreamOutput* stream)
{
    this->filename = filename;
    path_md5 = change_to_md5_path(filename);
    path_lz = change_to_lz_path(filename);

    active_stream = stream;
    stream->set_transferring(true);
    is_download = true;
    md5 = MD5();
    md5_str[0] = 0;
    last_len = 0;

    f_md5 = fopen(path_md5.c_str(), "rb");
    if (f_md5 != nullptr) {
        fgets(md5_str, sizeof(md5_str), f_md5);
        fclose(f_md5);
        f_md5 = nullptr;
        open_send_file();
        return;
    }

    fd = fopen(filename.c_str(), "rb");
    if (fd == nullptr) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        stream->printf("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        finish(false);
        return;
    }
    enter(DOWN_HASH);
}

void FileTransfer::enter(Phase p)
{
    phase = p;
    retries = 0;
    last_byte = us_ticker_read();
    up_decoder.reset();
    down_decoder.reset();
}

void FileTransfer::service()
{
    if (phase == IDLE) return;

    if (phase == UP_UNPACK) { unpack_step(); return; }
    if (phase == DOWN_HASH) { hash_step(); return; }

    char* buf = nullptr;
    int n = 0;
    if (active_stream->ready() && (n = active_stream->gets(&buf, 0)) > 0) {
        last_byte = us_ticker_read();
        Frame::Decoder& dec = (phase == DOWN_WAIT) ? down_decoder : up_decoder;
        for (int i = 0; i < n; i++) {
            if (!dec.feed((uint8_t)buf[i])) continue;
            if (phase == DOWN_WAIT) down_frame(dec); else up_frame(dec);
            if (phase == IDLE || phase == UP_UNPACK || phase == DOWN_HASH) return;
        }
        return;
    }

    check_deadline();
}

void FileTransfer::check_deadline()
{
    if (us_ticker_read() - last_byte < IDLE_TIMEOUT_MS * 1000) return;

    if (++retries > MAX_RETRIES) {
        active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        active_stream->printf(is_download ? "Error: download timed out\r\n" : "Error: upload timed out\r\n");
        finish(false);
        return;
    }

    last_byte = us_ticker_read();
    if (is_download) return;   // the client drives a download: it is only late, not lost

    up_decoder.reset();
    switch (phase) {
        case UP_MD5:  active_stream->send(Frame::FILE_MD5, "ok\r\n", 4); break;
        case UP_VIEW: active_stream->send(Frame::FILE_VIEW, "ok\r\n", 4); break;
        case UP_DATA: send_seq(Frame::FILE_DATA, seq); break;
        default: break;
    }
}

void FileTransfer::up_frame(const Frame::Decoder& dec)
{
    const uint8_t* p = dec.payload();
    uint16_t plen = dec.length();

    switch (dec.type()) {
        case Frame::FILE_CAN:
            active_stream->printf("Info: upload canceled by client\r\n");
            finish(false);
            break;

        case Frame::FILE_MD5:
            if (phase == UP_MD5 && plen >= 32) {
                received_md5.assign((const char*)p, 32);
                enter(UP_VIEW);
                active_stream->send(Frame::FILE_VIEW, "ok\r\n", 4);
            }
            // anything else is a stray or short frame: silence, and the deadline re-asks
            break;

        case Frame::FILE_VIEW:
            if (phase == UP_VIEW && plen >= 6) {
                total_packets = be32(p);
                packet_size = be16(p + 4);
                if (total_packets == 0 || packet_size == 0) {
                    active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                    active_stream->printf("Error: bad file view\r\n");
                    finish(false);
                    return;
                }
                if ((size_t)packet_size + 4 > sizeof(xbuff)) {
                    active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                    active_stream->printf("Error: packet size %u exceeds %u\r\n",
                                          (unsigned)packet_size, (unsigned)sizeof(xbuff));
                    finish(false);
                    return;
                }
                enter(UP_DATA);
                seq = 1;
            }
            if (phase == UP_DATA) send_seq(Frame::FILE_DATA, seq);
            break;

        case Frame::FILE_DATA: {
            if (phase != UP_DATA || plen < 4) break;

            uint32_t rx_seq = be32(p);
            if (rx_seq != seq) break;

            uint32_t dlen = plen - 4;
            if (fwrite(p + 4, 1, dlen, fd) != dlen) {
                active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                active_stream->printf("Error: file write error\r\n");
                finish(false);
                return;
            }
            if (hash_on_wire) md5.update(p + 4, dlen);

            retries = 0;
            file_size += dlen;
            if (seq < total_packets) {
                seq++;
                send_seq(Frame::FILE_DATA, seq);
            } else {
                last_packet();
            }
            break;
        }

        default:
            break;
    }
}

void FileTransfer::last_packet()
{
    fclose(fd);
    fd = nullptr;

    if (hash_on_wire) {
        computed_md5 = md5.finalize().hexdigest();
        if (received_md5 != computed_md5) {
            active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
            active_stream->printf("Error: MD5 verification failed\r\n");
            finish(false);
            return;
        }
    }
    active_stream->send(Frame::FILE_END, "ok\r\n", 4);

    if (!is_lz) {
        finish(true);
        return;
    }

    dest = filename.substr(0, filename.find(".lz"));
    packed_size = file_size;
    if (packed_size < BLOCK_HEADER_SIZE + 2) {
        active_stream->printf("Error: decompression failed\r\n");
        finish(false, true);
        return;
    }

    f_in = fopen(datafile.c_str(), "rb");
    f_out = fopen((dest + PART_SUFFIX).c_str(), "wb");
    if (f_in == nullptr || f_out == nullptr) {
        active_stream->printf("Error: Failed to open files for decompression!\r\n");
        unpack_finish(false);
        return;
    }

    memset(&qlz, 0x00, sizeof(qlz));
    unpack_off = 0;
    block_size = 0;
    block_num = 0;
    sum16 = 0;
    enter(UP_UNPACK);
}

void FileTransfer::unpack_step()
{
    uint8_t hdr[BLOCK_HEADER_SIZE];
    uint8_t tail[2];

    if (unpack_off >= packed_size - 2) {
        if (fread(tail, 1, 2, f_in) != 2) { unpack_finish(false); return; }
        if (sum16 != ((tail[0] << 8) + tail[1])) { unpack_finish(false); return; }
        computed_md5 = md5.finalize().hexdigest();
        unpack_finish(true);
        return;
    }

    if (fread(hdr, 1, BLOCK_HEADER_SIZE, f_in) != BLOCK_HEADER_SIZE) { unpack_finish(false); return; }
    block_size = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) | ((uint32_t)hdr[2] << 8) | hdr[3];
    if (!block_size || block_size > XBUFF_SIZE) { unpack_finish(false); return; }
    if (fread(xbuff, 1, block_size, f_in) != block_size) { unpack_finish(false); return; }

    uint32_t out = qlz_decompress((const char*)xbuff, lzbuff, &qlz);
    if (!out) { unpack_finish(false); return; }

    for (uint32_t j = 0; j < out; j++) sum16 += lzbuff[j];
    md5.update((const unsigned char *)lzbuff, out);
    if (fwrite(lzbuff, 1, out, f_out) != out) { unpack_finish(false); return; }

    block_num++;
    active_stream->printf("#Info: decompart = %lu\r\n", (unsigned long)block_num);
    unpack_off += BLOCK_HEADER_SIZE + block_size;
}

void FileTransfer::unpack_finish(bool ok)
{
    if (f_in != nullptr) { fclose(f_in); f_in = nullptr; }
    if (f_out != nullptr) { fclose(f_out); f_out = nullptr; }

    string part = dest + PART_SUFFIX;
    if (!ok) {
        active_stream->printf("Error: decompression failed\r\n");
        remove(part.c_str());
        finish(false, true);
        return;
    }

    if (received_md5 != computed_md5) {
        active_stream->printf("Error: MD5 verification failed\r\n");
        remove(part.c_str());
        finish(false, true);
        return;
    }

    remove(dest.c_str());
    if (rename(part.c_str(), dest.c_str()) != 0) {
        active_stream->printf("Error: could not rename %s\r\n", part.c_str());
        remove(part.c_str());
        finish(false, true);
        return;
    }
    finish(true);
}

void FileTransfer::hash_step()
{
    size_t n = fread(xbuff, 1, sizeof(xbuff), fd);
    if (n > 0) md5.update(xbuff, n);
    if (!feof(fd)) return;

    strcpy(md5_str, md5.finalize().hexdigest().c_str());
    fclose(fd);
    fd = nullptr;
    open_send_file();
}

void FileTransfer::open_send_file()
{
    fd = fopen(path_lz.c_str(), "rb");
    if (fd == nullptr) fd = fopen(filename.c_str(), "rb");
    if (fd == nullptr) {
        active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        active_stream->printf("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        finish(false);
        return;
    }

    fseek(fd, 0, SEEK_END);
    long size = ftell(fd);
    rewind(fd);
    total_packets = ((uint32_t)size + DOWNLOAD_CHUNK - 1) / DOWNLOAD_CHUNK;

    last_len = Frame::encode(Frame::FILE_MD5, md5_str, 32, xbuff);
    active_stream->puts((char*)xbuff, last_len);

    enter(DOWN_WAIT);
}

void FileTransfer::down_frame(const Frame::Decoder& dec)
{
    const uint8_t* p = dec.payload();
    uint16_t plen = dec.length();
    retries = 0;   // any complete frame is proof the client is there

    switch (dec.type()) {
        case Frame::FILE_MD5:
            last_len = Frame::encode(Frame::FILE_MD5, md5_str, 32, xbuff);
            active_stream->puts((char*)xbuff, last_len);
            break;

        case Frame::FILE_VIEW: {
            uint8_t v[6] = { (uint8_t)(total_packets >> 24), (uint8_t)(total_packets >> 16), (uint8_t)(total_packets >> 8), (uint8_t)total_packets,
                             (uint8_t)(DOWNLOAD_CHUNK >> 8), (uint8_t)DOWNLOAD_CHUNK };
            last_len = Frame::encode(Frame::FILE_VIEW, v, sizeof(v), xbuff);
            active_stream->puts((char*)xbuff, last_len);
            break;
        }

        case Frame::FILE_DATA: {
            if (plen < 4) break;
            uint32_t rx_seq = be32(p);
            if (rx_seq < 1 || rx_seq > total_packets) break;

            fseek(fd, (long)(rx_seq - 1) * DOWNLOAD_CHUNK, SEEK_SET);
            uint8_t* data = xbuff + 5 + 4;
            size_t n = fread(data, 1, DOWNLOAD_CHUNK, fd);
            if (n == 0) {
                active_stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                active_stream->printf("Error: file read error\r\n");
                finish(false);
                return;
            }

            uint8_t s[4] = { (uint8_t)(rx_seq >> 24), (uint8_t)(rx_seq >> 16), (uint8_t)(rx_seq >> 8), (uint8_t)rx_seq };
            memcpy(xbuff + 5, s, 4);
            uint16_t flen = 4 + n + 3;
            xbuff[0] = Frame::HEADER >> 8; xbuff[1] = Frame::HEADER & 0xFF;
            xbuff[2] = flen >> 8;          xbuff[3] = flen & 0xFF;
            xbuff[4] = Frame::FILE_DATA;
            uint16_t c = Frame::crc16(0, xbuff + 2, flen);
            uint8_t* t = xbuff + 5 + 4 + n;
            t[0] = c >> 8; t[1] = c & 0xFF; t[2] = Frame::FOOTER >> 8; t[3] = Frame::FOOTER & 0xFF;
            last_len = 5 + 4 + n + 4;
            active_stream->puts((char*)xbuff, last_len);
            break;
        }

        case Frame::FILE_RETRY:
            if (last_len) active_stream->puts((char*)xbuff, last_len);
            break;

        case Frame::FILE_END:
            active_stream->send(Frame::FILE_END, "ok\r\n", 4);
            finish(true);
            break;

        case Frame::FILE_CAN:
            active_stream->printf("Info: download canceled by client\r\n");
            finish(false);
            break;

        default:
            break;
    }
}

void FileTransfer::finish(bool ok, bool keep_data, bool quiet)
{
    if (fd != nullptr) { fclose(fd); fd = nullptr; }
    if (!ok && !is_download && !keep_data) remove(datafile.c_str());

    if (ok && !is_download && !is_lz) {
        remove(filename.c_str());
        ok = rename(datafile.c_str(), filename.c_str()) == 0;
        if (!ok && !quiet) {
            active_stream->printf("Error: could not rename %s\r\n", datafile.c_str());
        }
        if (!ok) remove(datafile.c_str());
    }

    if (f_md5 != nullptr) {
        if (ok) fwrite(computed_md5.c_str(), 1, computed_md5.size(), f_md5);
        fclose(f_md5);
        f_md5 = nullptr;
        if (!ok) remove(path_md5.c_str());
    }

    if (!quiet) {
        if (is_download) {
            active_stream->printf(ok ? "Info: Download success: %s.\r\n"
                                     : "Download failed for file: %s.\r\n", filename.c_str());
        } else if (ok) {
            scripts.file_changed(filename.c_str());
            active_stream->printf("Info: upload success: %s.\r\n", filename.c_str());
        } else {
            active_stream->printf("Upload failed for file: %s.\r\n", filename.c_str());
        }
    }

    phase = IDLE;
    is_download = false;
    is_lz = false;
    hash_on_wire = false;
    want_md5_file = false;
    received_md5.clear();
    computed_md5.clear();
    filename.clear();
    datafile.clear();
    path_md5.clear();
    path_lz.clear();
    dest.clear();
    total_packets = 0;
    seq = 1;
    file_size = 0;
    packet_size = 0;
    md5_str[0] = 0;
    last_len = 0;
    retries = 0;
    active_stream->set_transferring(false);
    active_stream = nullptr;
}
