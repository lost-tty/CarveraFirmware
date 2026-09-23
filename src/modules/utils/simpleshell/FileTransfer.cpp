#include "FileTransfer.h"
#include "Frame.h"
#include "libs/Kernel.h"
#include "Conveyor.h"
#include "quicklz.h"
#include "utils.h"
#include "mbed.h"
#include "Scripts.h"
#include "Source.h"
#include "md5.h"
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

using namespace std;

static inline uint32_t be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline uint16_t be16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | p[1]; }

int FileTransfer::in(StreamOutput* stream, uint32_t timeout_ms)
{
    uint32_t start = us_ticker_read();
    for (;;) {
        if (pend_len > 0) {
            pend_len--;
            return (uint8_t)*pend++;
        }
        if (stream->ready()) {
            char* buf;
            int n = stream->gets(&buf, 0);
            if (n > 0) {
                pend = buf;
                pend_len = n;
                continue;
            }
        }
        if (us_ticker_read() - start >= timeout_ms * 1000) return -1;
        // feeds the watchdog
        THEKERNEL->call_event(ON_IDLE);
        vTaskDelay(0);
    }
}

int FileTransfer::read_header(StreamOutput* stream, uint32_t timeout_ms, uint16_t& plen, uint16_t& crc)
{
    int prev = -1;
    for (;;) {
        int c = in(stream, timeout_ms);
        if (c < 0) return -1;
        if (prev == (Frame::HEADER >> 8) && c == (Frame::HEADER & 0xFF)) break;
        prev = c;
    }
    uint8_t hdr[3];
    for (int i = 0; i < 3; i++) {
        int c = in(stream, BYTE_TIMEOUT_MS);
        if (c < 0) return -1;
        hdr[i] = c;
    }
    uint16_t len = be16(hdr);
    if (len < 3) return -1;
    plen = len - 3;
    crc = Frame::crc16(0, hdr, 3);
    return hdr[2];
}

bool FileTransfer::read_trailer(StreamOutput* stream, uint16_t crc)
{
    uint8_t t[4];
    for (int i = 0; i < 4; i++) {
        int c = in(stream, BYTE_TIMEOUT_MS);
        if (c < 0) return false;
        t[i] = c;
    }
    return be16(t) == crc && be16(t + 2) == Frame::FOOTER;
}

bool FileTransfer::read_payload(StreamOutput* stream, uint8_t* dst, uint16_t plen, uint16_t crc)
{
    for (uint16_t i = 0; i < plen; i++) {
        int c = in(stream, BYTE_TIMEOUT_MS);
        if (c < 0) return false;
        dst[i] = c;
    }
    crc = Frame::crc16(crc, dst, plen);
    return read_trailer(stream, crc);
}

void FileTransfer::skip_payload(StreamOutput* stream, uint16_t plen)
{
    for (uint32_t i = 0; i < (uint32_t)plen + 4; i++) {
        if (in(stream, BYTE_TIMEOUT_MS) < 0) return;
    }
}

void FileTransfer::send_seq(StreamOutput* stream, uint8_t type, uint32_t seq)
{
    uint8_t p[4] = { (uint8_t)(seq >> 24), (uint8_t)(seq >> 16), (uint8_t)(seq >> 8), (uint8_t)seq };
    stream->send(type, p, sizeof(p));
}

bool FileTransfer::upload(const std::string& filename, StreamOutput* stream)
{
    if (sources.active()) {
        stream->printf("error:busy, a job or script is running\r\n");
        return false;
    }
    enum { WAIT_MD5, WAIT_VIEW, DATA } state = WAIT_MD5;
    uint32_t total_packets = 0, seq = 1, file_size = 0;
    int retries = 0;
    bool ok = false;
    std::string received_md5, computed_md5;
    MD5 md5;

    string md5_filename = change_to_md5_path(filename);
    string lzfilename = change_to_lz_path(filename);
    check_and_make_path(md5_filename);
    check_and_make_path(lzfilename);

    pend_len = 0;

    if (!THECONVEYOR.is_idle()) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        return false;
    }

    Claim claim(stream);

    // .lz uploads land in the .lz shadow directory and are decompressed to filename afterwards
    bool is_lz = filename.find(".lz") != string::npos;
    string datafile = filename;
    if (is_lz) {
        datafile = lzfilename.substr(0, lzfilename.rfind(".lz"));
        md5_filename = md5_filename.substr(0, md5_filename.find(".lz"));
    }
    bool want_md5_file = filename.find("firmware.bin") == string::npos;

    FILE* fd = fopen(datafile.c_str(), "wb");
    FILE* fd_md5 = want_md5_file ? fopen(md5_filename.c_str(), "wb") : NULL;
    if (fd == NULL || (want_md5_file && fd_md5 == NULL)) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        stream->printf("Error: failed to open file [%s]!\r\n", fd == NULL ? datafile.substr(0, 30).c_str() : md5_filename.substr(0, 30).c_str());
        goto done;
    }

    for (;;) {
        uint16_t plen, crc;
        int type = read_header(stream, IDLE_TIMEOUT_MS, plen, crc);

        if (type < 0) {
            if (++retries > MAX_RETRIES) {
                stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                stream->printf("Error: upload timed out\r\n");
                goto done;
            }
            switch (state) {
                case WAIT_MD5:  stream->send(Frame::FILE_MD5, "ok\r\n", 4); break;
                case WAIT_VIEW: stream->send(Frame::FILE_VIEW, "ok\r\n", 4); break;
                case DATA:      send_seq(stream, Frame::FILE_DATA, seq); break;
            }
            continue;
        }

        if (type == Frame::FILE_DATA) {
            if (state != DATA || plen < 4) { skip_payload(stream, plen); continue; }

            uint8_t s[4];
            bool got = true;
            for (int i = 0; i < 4; i++) { int c = in(stream, BYTE_TIMEOUT_MS); if (c < 0) { got = false; break; } s[i] = c; }
            if (!got) continue;
            crc = Frame::crc16(crc, s, 4);
            uint32_t rx_seq = be32(s);
            uint32_t dlen = plen - 4;

            if (rx_seq != seq) {
                // stale duplicate: do not re-request, the client answers every request
                skip_payload(stream, dlen);
                continue;
            }

            if (dlen > sizeof(xbuff)) {
                skip_payload(stream, dlen);
                stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                stream->printf("Error: packet size %lu exceeds %u\r\n", (unsigned long)dlen, (unsigned)sizeof(xbuff));
                goto done;
            }

            // whole packet first, then the SD write: an SD write mid-packet would overrun the UART FIFO
            bool good = true;
            for (uint32_t i = 0; i < dlen; i++) {
                int c = in(stream, BYTE_TIMEOUT_MS);
                if (c < 0) { good = false; break; }
                xbuff[i] = c;
            }
            if (good) {
                crc = Frame::crc16(crc, xbuff, dlen);
                good = read_trailer(stream, crc);
            }
            if (!good) {
                if (++retries > MAX_RETRIES) {
                    stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                    stream->printf("Error: too many bad packets\r\n");
                    goto done;
                }
                send_seq(stream, Frame::FILE_DATA, seq);
                continue;
            }

            if (fwrite(xbuff, 1, dlen, fd) != dlen) {
                stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                stream->printf("Error: file write error\r\n");
                goto done;
            }
            md5.update(xbuff, dlen);

            retries = 0;
            file_size += dlen;
            if (seq < total_packets) {
                seq++;
                send_seq(stream, Frame::FILE_DATA, seq);
            } else {
                fflush(fd);
                computed_md5 = md5.finalize().hexdigest();
                if (received_md5 != computed_md5) {
                    stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                    stream->printf("Error: MD5 verification failed\r\n");
                    goto done;
                }
                stream->send(Frame::FILE_END, "ok\r\n", 4);
                ok = true;
                goto done;
            }
            continue;
        }

        if (plen > sizeof(xbuff)) { skip_payload(stream, plen); continue; }
        if (!read_payload(stream, xbuff, plen, crc)) continue;

        switch (type) {
            case Frame::FILE_CAN:
                stream->printf("Info: upload canceled by client\r\n");
                goto done;

            case Frame::FILE_MD5:
                if (state == WAIT_MD5 && plen >= 32) {
                    received_md5.assign((char*)xbuff, 32);
                    state = WAIT_VIEW;
                    retries = 0;
                }
                stream->send(Frame::FILE_VIEW, "ok\r\n", 4);
                break;

            case Frame::FILE_VIEW:
                if (state == WAIT_VIEW && plen >= 6) {
                    total_packets = be32(xbuff);
                    uint16_t packet_size = be16(xbuff + 4);
                    if (total_packets == 0 || packet_size == 0) {
                        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                        stream->printf("Error: bad file view\r\n");
                        goto done;
                    }
                    state = DATA;
                    seq = 1;
                    retries = 0;
                }
                if (state == DATA) send_seq(stream, Frame::FILE_DATA, seq);
                break;

            default:
                break;
        }
    }

done:
    if (fd != NULL) fclose(fd);
    if (fd_md5 != NULL) {
        if (ok) fwrite(computed_md5.c_str(), 1, computed_md5.size(), fd_md5);
        fclose(fd_md5);
        if (!ok) remove(md5_filename.c_str());
    }
    if (!ok) remove(datafile.c_str());

    claim.release(); // decompression is slow and reads nothing from the stream

    if (ok && is_lz) {
        string dest = filename.substr(0, filename.find(".lz"));
        ok = decompress(datafile, dest, file_size, stream);
    }
    if (ok) scripts.file_changed(filename.c_str());
    return ok;
}

bool FileTransfer::download(const std::string& filename, StreamOutput* stream)
{
    char md5_str[33] = { 0 };
    uint8_t ctrl[64];
    size_t last_len = 0;            // last frame we sent sits in xbuff, for FILE_RETRY
    int idle = 0;
    bool ok = false;

    string md5_filename = change_to_md5_path(filename);
    string lz_filename = change_to_lz_path(filename);

    pend_len = 0;

    if (!THECONVEYOR.is_idle()) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        return false;
    }

    Claim claim(stream);

    FILE* fd = fopen(md5_filename.c_str(), "rb");
    if (fd != NULL) {
        fgets(md5_str, sizeof(md5_str), fd);
        fclose(fd);
    } else {
        fd = fopen(filename.c_str(), "rb");
        if (fd == NULL) {
            stream->send(Frame::FILE_CAN, "ok\r\n", 4);
            stream->printf("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
            goto done;
        }
        MD5 md5;
        do {
            size_t n = fread(xbuff, 1, sizeof(xbuff), fd);
            if (n > 0) md5.update(xbuff, n);
            THEKERNEL->call_event(ON_IDLE);
        } while (!feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fclose(fd);
    }

    // send the compressed copy if there is one
    fd = fopen(lz_filename.c_str(), "rb");
    if (fd == NULL) fd = fopen(filename.c_str(), "rb");
    if (fd == NULL) {
        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
        stream->printf("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        goto done;
    }

    {
        fseek(fd, 0, SEEK_END);
        long file_size = ftell(fd);
        rewind(fd);
        uint32_t total_packets = (file_size + DOWNLOAD_CHUNK - 1) / DOWNLOAD_CHUNK;

        last_len = Frame::encode(Frame::FILE_MD5, md5_str, 32, xbuff);
        stream->puts((char*)xbuff, last_len);

        for (;;) {
            uint16_t plen, crc;
            int type = read_header(stream, IDLE_TIMEOUT_MS, plen, crc);
            if (type < 0) {
                if (++idle > MAX_RETRIES) {
                    stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                    stream->printf("Error: download timed out\r\n");
                    goto done;
                }
                continue;
            }
            if (plen > sizeof(ctrl)) { skip_payload(stream, plen); continue; }
            if (!read_payload(stream, ctrl, plen, crc)) continue;
            idle = 0;

            switch (type) {
                case Frame::FILE_MD5:
                    last_len = Frame::encode(Frame::FILE_MD5, md5_str, 32, xbuff);
                    stream->puts((char*)xbuff, last_len);
                    break;

                case Frame::FILE_VIEW: {
                    uint8_t v[6] = { (uint8_t)(total_packets >> 24), (uint8_t)(total_packets >> 16), (uint8_t)(total_packets >> 8), (uint8_t)total_packets,
                                     (uint8_t)(DOWNLOAD_CHUNK >> 8), (uint8_t)DOWNLOAD_CHUNK };
                    last_len = Frame::encode(Frame::FILE_VIEW, v, sizeof(v), xbuff);
                    stream->puts((char*)xbuff, last_len);
                    break;
                }

                case Frame::FILE_DATA: {
                    if (plen < 4) break;
                    uint32_t seq = be32(ctrl);
                    if (seq < 1 || seq > total_packets) break;
                    fseek(fd, (seq - 1) * DOWNLOAD_CHUNK, SEEK_SET);
                    uint8_t* data = xbuff + 5 + 4;
                    size_t n = fread(data, 1, DOWNLOAD_CHUNK, fd);
                    if (n == 0) {
                        stream->send(Frame::FILE_CAN, "ok\r\n", 4);
                        stream->printf("Error: file read error\r\n");
                        goto done;
                    }
                    uint8_t s[4] = { (uint8_t)(seq >> 24), (uint8_t)(seq >> 16), (uint8_t)(seq >> 8), (uint8_t)seq };
                    memcpy(xbuff + 5, s, 4);
                    uint16_t flen = 4 + n + 3;
                    xbuff[0] = Frame::HEADER >> 8; xbuff[1] = Frame::HEADER & 0xFF;
                    xbuff[2] = flen >> 8;          xbuff[3] = flen & 0xFF;
                    xbuff[4] = Frame::FILE_DATA;
                    uint16_t c = Frame::crc16(0, xbuff + 2, flen);
                    uint8_t* t = xbuff + 5 + 4 + n;
                    t[0] = c >> 8; t[1] = c & 0xFF; t[2] = Frame::FOOTER >> 8; t[3] = Frame::FOOTER & 0xFF;
                    last_len = 5 + 4 + n + 4;
                    stream->puts((char*)xbuff, last_len);
                    break;
                }

                case Frame::FILE_RETRY:
                    if (last_len) stream->puts((char*)xbuff, last_len);
                    break;

                case Frame::FILE_END:
                    stream->send(Frame::FILE_END, "ok\r\n", 4);
                    ok = true;
                    goto done;

                case Frame::FILE_CAN:
                    stream->printf("Info: download canceled by client\r\n");
                    goto done;

                default:
                    break;
            }
        }
    }

done:
    if (fd != NULL) fclose(fd);
    return ok;
}

bool FileTransfer::decompress(const std::string& sfilename, const std::string& dfilename, uint32_t sfilesize, StreamOutput* stream)
{
    uint16_t u16Sum = 0;
    uint8_t u8ReadBuffer_hdr[BLOCK_HEADER_SIZE] = { 0 };
    uint32_t u32DcmprsSize = 0, u32BlockSize = 0, u32BlockNum = 0, u32TotalDcmprsSize = 0, i = 0, j = 0, k = 0;
    qlz_state_decompress s_stDecompressState;

    FILE* f_in = fopen(sfilename.c_str(), "rb");
    FILE* f_out = fopen(dfilename.c_str(), "wb");

    if (f_in == NULL || f_out == NULL) {
        stream->printf("Error: Failed to open files for decompression!\r\n");
        goto _exit;
    }

    memset(&s_stDecompressState, 0x00, sizeof(qlz_state_decompress));

    for (i = 0; i < sfilesize - 2; i += BLOCK_HEADER_SIZE + u32BlockSize) {
        fread(u8ReadBuffer_hdr, sizeof(char), BLOCK_HEADER_SIZE, f_in);
        u32BlockSize = u8ReadBuffer_hdr[0] * (1 << 24) + u8ReadBuffer_hdr[1] * (1 << 16) + u8ReadBuffer_hdr[2] * (1 << 8) + u8ReadBuffer_hdr[3];

        if (!u32BlockSize || u32BlockSize > XBUFF_SIZE) {
            goto _exit;
        }

        fread(xbuff, sizeof(char), u32BlockSize, f_in);
        u32DcmprsSize = qlz_decompress((const char*)xbuff, lzbuff, &s_stDecompressState);
        if (!u32DcmprsSize) {
            goto _exit;
        }

        for (j = 0; j < u32DcmprsSize; j++) {
            u16Sum += lzbuff[j];
        }

        fwrite(lzbuff, sizeof(char), u32DcmprsSize, f_out);
        u32TotalDcmprsSize += u32DcmprsSize;
        u32BlockNum += 1;
        if (++k > 10) {
            k = 0;
            THEKERNEL->call_event(ON_IDLE);
        }
        stream->printf("#Info: decompart = %lu\r\n", u32BlockNum);
    }

    fread(xbuff, sizeof(char), 2, f_in);

    if (u16Sum != ((xbuff[0] << 8) + xbuff[1])) {
        goto _exit;
    }

    fclose(f_in);
    fclose(f_out);

    stream->printf("#Info: decompart = %lu\r\n", u32BlockNum);
    return true;

_exit:
    if (f_in != NULL) fclose(f_in);
    if (f_out != NULL) fclose(f_out);
    stream->printf("Error: decompression failed\r\n");
    return false;
}
