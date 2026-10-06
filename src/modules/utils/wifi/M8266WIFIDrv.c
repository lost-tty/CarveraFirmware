/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

// Driver for the M8266 SPI WiFi module (an ESP8266 HSPI slave) on an LPC17xx SSP.
// Wire-compatible with the vendor's M8266WIFIDrv_LPC17xx.a V1.2.0-6, which it replaces.
//
// The module exposes an 8-bit status register (SPI cmd 0x01 write, 0x04 read) and a
// 32-byte buffer (0x02 write, 0x03 read). Requests and replies are frames in that
// buffer: cmd, len, payload[len], sum of the preceding bytes. Behaviour depends on the
// module firmware version, which is read once in M8266HostIf_SPI_Select.

#include <stdint.h>
#include <string.h>
#include "M8266WIFIDrv.h"

typedef struct {
    volatile uint32_t CR0, CR1, DR, SR, CPSR;
} Ssp;

enum { SR_TFE = 1, SR_TNF = 2, SR_RNE = 4, SR_BSY = 16 };

enum {
    ST_READY = 0x01,        // module idle, accepts a request
    ST_REQ = 0x02,          // host raised: request in the buffer; module clears on pickup
    ST_DATA_END = 0x04,     // host raised at the end of a payload transfer
    ST_RX_DATA = 0x10,      // received payload waiting
    ST_CHUNK_REQ = 0x20,    // host raised: load the next payload chunk; module clears
    ST_LAST_CHUNK = 0x40,   // the loaded chunk is the payload's last
    ST_REPLY = 0x80,        // reply frame in the buffer
};

#define SPIN_MAX 60001

#define POLL_TIMEOUT_US 150000u
#define SPIN_US 200u

static Ssp* ssp;
static int8_t reread_after_write;     // modules >= 1.2.0-1 want a status read after each write
static char module_ver[24] = "1.1.1-1";

static const char drv_mcu[16] = "LPC17XX";
static const char drv_ver[10] = "V1.2.0-6";

// What an earlier transfer that only wrote (buf_write) left: the last bytes on the wire and
// their echoes in the receive FIFO.
static void drain(void)
{
    for (int n = 0; (ssp->SR & SR_BSY) && n < SPIN_MAX; n++) {}
    for (int n = 0; (ssp->SR & SR_RNE) && n < SPIN_MAX; n++)
        (void)ssp->DR;
}

// One transaction's n bytes: out's (zeros past head_len), the clock running without a gap
// with at most 8 in flight so the receive FIFO cannot overflow; the echoes from byte skip on
// go to in.
static void exchange(const uint8_t* head, uint8_t head_len, uint8_t* in, uint8_t skip,
                     uint8_t n)
{
    drain();
    uint8_t sent = 0, got = 0;
    int idle = 0;
    while (got < n && idle < SPIN_MAX) {
        if (sent < n && (uint8_t)(sent - got) < 8 && (ssp->SR & SR_TNF)) {
            ssp->DR = sent < head_len ? head[sent] : 0;
            sent++;
        }
        if (!(ssp->SR & SR_RNE)) {
            idle++;
            continue;
        }
        uint8_t v = ssp->DR;
        if (got >= skip)
            in[got - skip] = v;

        got++;
        idle = 0;
    }
}

static void wait_tnf(void)
{
    for (int n = 0; !(ssp->SR & SR_TNF) && n < SPIN_MAX; n++) {}
}

static uint8_t reg_read(void)
{
    static const uint8_t cmd[1] = { 0x04 };
    uint8_t v = 0;
    M8266HostIf_Set_SPI_nCS_Pin(0);
    exchange(cmd, 1, &v, 1, 2);
    M8266HostIf_Set_SPI_nCS_Pin(1);
    return v;
}

static void reg_write(uint8_t v)
{
    uint8_t cmd[2] = { 0x01, v };
    M8266HostIf_Set_SPI_nCS_Pin(0);
    exchange(cmd, 2, 0, 2, 2);
    M8266HostIf_Set_SPI_nCS_Pin(1);
}

static void buf_write(const uint8_t* p, uint8_t n)
{
    if (n == 0)
        return;

    if (n > 32)
        n = 32;

    M8266HostIf_Set_SPI_nCS_Pin(0);
    wait_tnf();
    ssp->DR = 0x02;
    wait_tnf();
    ssp->DR = 0x00;
    for (uint8_t i = 0; i < n; i++) {
        wait_tnf();
        ssp->DR = p[i];
    }
    for (int n = 0; ((ssp->SR & SR_BSY) || !(ssp->SR & SR_TFE)) && n < SPIN_MAX; n++) {}
    M8266HostIf_Set_SPI_nCS_Pin(1);
}

// Clocks out zeros and collects what comes back.
static void buf_read(uint8_t* p, uint8_t n)
{
    if (n == 0)
        return;

    if (n > 32)
        n = 32;

    static const uint8_t cmd[2] = { 0x03, 0x00 };
    M8266HostIf_Set_SPI_nCS_Pin(0);
    exchange(cmd, 2, p, 2, n + 2);
    M8266HostIf_Set_SPI_nCS_Pin(1);
}

static void delay_ms(uint16_t ms)
{
    if (ms)
        M8266HostIf_delay_ms(ms);
}

static u8 expired(uint32_t since, uint32_t limit_us)
{
    return M8266HostIf_now_us() - since >= limit_us;
}

// Between two status reads: none while an answer is likely to be quick, then a tick.
static void poll_pause(uint32_t since)
{
    if (M8266HostIf_now_us() - since >= SPIN_US)
        M8266HostIf_delay_ms(1);
}

static void fail(u16* status, uint8_t code)
{
    if (status)
        *status = (u16)(reg_read() << 8 | code);
}

// Two equal reads in 1..254, or the last read once the poll time runs out.
static uint8_t reg_read_stable(uint8_t prev)
{
    uint32_t t0 = M8266HostIf_now_us();
    for (;;) {
        uint8_t v = reg_read();
        if (v >= 1 && v <= 254 && v == prev)
            return v;

        if (expired(t0, POLL_TIMEOUT_US))
            return v;

        poll_pause(t0);
        prev = v;
    }
}

static void flag_clear(uint8_t m, uint8_t delay)
{
    uint8_t s = reg_read();
    if (!(s & m))
        return;

    s = reg_read_stable(s);
    if (!(s & m))
        return;

    reg_write(s & ~m & ~ST_DATA_END);
    if (delay)
        M8266HostIf_delay_us(delay);
}

static void flag_set(uint8_t m, uint8_t stabilize, uint8_t delay)
{
    const uint8_t always = ST_DATA_END | ST_CHUNK_REQ;
    uint8_t s = reg_read();
    if ((s & m) && !(m & always))
        return;

    if (stabilize) {
        s = reg_read_stable(s);
        if ((s & m) && !(m & always))
            return;
    }
    uint8_t w = s | m;
    if (m != ST_DATA_END)
        w &= ~ST_DATA_END;

    reg_write(w);
    if (reread_after_write)
        reg_read();

    M8266HostIf_delay_us(delay);
}

static u8 wait_ready(u16* status)
{
    uint32_t t0 = M8266HostIf_now_us();
    for (;;) {
        uint8_t s = reg_read();
        if (s != 0xff && (s & ST_READY))
            return 1;

        if (expired(t0, POLL_TIMEOUT_US))
            break;

        poll_pause(t0);
    }
    fail(status, 0x10);
    return 0;
}

static u8 wait_picked_up(u16* status)
{
    uint32_t t0 = M8266HostIf_now_us();
    for (;;) {
        uint8_t s = reg_read();
        if (!s) {
            s = reg_read();
            if (!s)
                s = reg_read();
        }
        if (!(s & ST_REQ))
            return 1;

        if (expired(t0, POLL_TIMEOUT_US))
            break;

        poll_pause(t0);
    }
    fail(status, 0x11);
    return 0;
}

// Compares the module firmware version against v, like strcmp on signed chars.
// 'S' and 'T' end a version and are cut from the module's copy for good.
static int ver_cmp(const char* v)
{
    for (uint8_t i = 0;; i++) {
        if (module_ver[i] == 'S' || module_ver[i] == 'T')
            module_ver[i] = 0;

        int8_t a = module_ver[i], b = v[i];
        if (a == 0)
            return b < 0 ? 1 : b == 0 ? 0 : -1;

        if (b == 0 || a != b)
            return a > b ? 1 : -1;
    }
}

static u8 module_older(const char* v, u16* status)
{
    if (ver_cmp(v) >= 0)
        return 0;

    fail(status, 0x77);
    return 1;
}

// Waits for the module and raises the request flag.
static u8 ready(u16* status)
{
    if (!wait_ready(status))
        return 0;

    flag_clear(ST_REPLY, 10);
    flag_set(ST_REQ, 1, 10);
    return 1;
}

// Clears the status, checks the module version (none for NULL) and waits for the module.
static u8 begin(const char* min_ver, u16* status)
{
    if (status)
        *status = 0;

    if (min_ver && module_older(min_ver, status))
        return 0;

    return ready(status);
}

static void seal(uint8_t* f)
{
    uint8_t sum = 0;
    for (int i = 0; i < f[1] + 2; i++)
        sum += f[i];

    f[f[1] + 2] = sum;
}

// Sends the frame in f, waits for the reply and leaves it in f.
static u8 transact(uint8_t* f, uint8_t tx_len, uint8_t rx_len, uint16_t pre_wait_ms,
                   uint16_t timeout_ms, u16* status)
{
    uint8_t cmd = f[0];
    buf_write(f, tx_len);
    M8266HostIf_delay_us(10);
    delay_ms(pre_wait_ms);
    if (!wait_picked_up(status))
        return 0;

    uint32_t t0 = M8266HostIf_now_us();
    uint32_t limit = POLL_TIMEOUT_US + (uint32_t)timeout_ms * 1000;
    for (;;) {
        uint8_t s = reg_read();
        if (s >= ST_REPLY && s != 0xff)
            break;

        if (expired(t0, limit)) {
            fail(status, 0x16);
            return 0;
        }

        poll_pause(t0);
    }

    if (status)
        *status = 0;

    buf_read(f, rx_len);
    M8266HostIf_delay_us(10);
    flag_clear(ST_REPLY, 0);

    // a length that runs past the bytes read is a broken frame as much as a bad sum
    if (f[1] + 3 > rx_len) {
        fail(status, 0x30);
        return 0;
    }

    uint8_t sum = 0;
    for (int i = 0; i < f[1] + 2; i++)
        sum += f[i];

    if (sum != f[f[1] + 2]) {
        fail(status, 0x30);
        return 0;
    }
    // Long host-name lookups (0x36) are answered as 0x25.
    uint8_t rc = f[0] & 0x7f;
    if (rc != cmd && !(cmd == 0x36 && rc == 0x25)) {
        fail(status, 0x31);
        return 0;
    }
    // Newer modules refuse a request with 'x' and a reason code.
    if (module_older("1.1.6-N", NULL) || f[2] != 'x')
        return 1;

    if (!status)
        return 0;

    *status = (u16)(reg_read() << 8);
    switch (f[3]) {
    case 0: return 1;
    case 1: *status |= 0x45; break;
    case 2: *status |= 0x46; break;
    case 3: *status |= 0x47; break;
    default: *status |= 0x48; break;
    }
    return 0;
}

static u8 set_spi_mode(uint8_t mode, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[36];
    f[0] = 0x42;
    f[1] = 27;
    f[2] = mode;
    memcpy(&f[3], drv_mcu, 16);
    memcpy(&f[19], drv_ver, 10);
    seal(f);
    return transact(f, 30, 32, 0, 0, status) != 0;
}

u8 M8266HostIf_SPI_Select(uint32_t spi_base_addr, uint32_t spi_clock, u16* status)
{
    ssp = (Ssp*)spi_base_addr;
    if (status)
        *status = 0;

    // Negotiate at a slow clock with CPHA=0; above 20 MHz the module wants CPHA=1.
    uint32_t cpsr = ssp->CPSR;
    ssp->CPSR = 128;
    ssp->CR0 &= ~0x80u;
    uint8_t fast = spi_clock >= 20000000;
    if (!set_spi_mode(fast ? 1 : 2, status))
        return 0;

    ssp->CPSR = cpsr;
    if (fast)
        ssp->CR0 |= 0x80;

    delay_ms(1);
    if (!M8266WIFI_SPI_Get_Module_Info(NULL, NULL, module_ver, status))
        return 0;

    reread_after_write = ver_cmp("1.2.0-1") >= 0;
    return 1;
}

u8 M8266WIFI_SPI_Interface_Communication_OK(u8* byte)
{
    static const uint8_t pattern[6] = { 0x41, 0x49, 0x4b, 0x5b, 0xdb, 0x41 };
    uint8_t v = reg_read();
    u8 ok = 0;
    if (v != 0xff && (v = reg_read()) != 0) {
        ok = 1;
        for (int i = 0; i < 6 && ok; i++) {
            reg_write(pattern[i]);
            M8266HostIf_delay_us(10);
            v = reg_read();
            ok = v == pattern[i];
        }
    }
    if (byte)
        *byte = v;

    return ok;
}

u32 M8266WIFI_SPI_Interface_Communication_Stress_Test(u32 max_times)
{
    u32 good = 0;
    for (u32 i = 0; i != max_times; i++) {
        uint8_t v = (i & 0xda) | ST_READY;
        reg_write(v);
        M8266HostIf_delay_us(module_older("1.2.0-1", NULL) ? 10 : 1);
        if (reg_read() == v)
            good++;
    }
    reg_write(0x41);
    M8266HostIf_delay_us(10);
    return good;
}

u8 M8266WIFI_SPI_Has_DataReceived(void)
{
    uint8_t s = reg_read();
    return s != 0xff && (s & ST_RX_DATA);
}

u8 M8266WIFI_SPI_Get_Module_Info(u32* module_id, u8* flash_size, char* fw_ver, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[36] = { 0x1f, 0x00, 0x1f };
    if (!transact(f, 3, 32, 0, 0, status))
        return 0;

    if (module_id)
        *module_id = (u32)f[2] << 24 | f[3] << 16 | f[4] << 8 | f[5];

    if (flash_size)
        *flash_size = f[6];

    if (fw_ver) {
        uint16_t i = 0;
        for (; i < f[7] && i != 23; i++)
            fw_ver[i] = f[8 + i];

        fw_ver[i] = 0;
    }
    return 1;
}

u8 M8266WIFI_SPI_Get_Opmode(u8* op_mode, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[36] = { 0x10, 0x00, 0x10 };
    if (!transact(f, 3, 32, 0, 0, status))
        return 0;

    if (op_mode)
        *op_mode = f[2];

    return 1;
}

// Sends a frame with an empty payload and leaves the reply in f.
static u8 send(uint8_t* f, uint16_t len, u16* status)
{
    seal(f);
    return transact(f, len, 32, 0, 0, status);
}

static u8 query(uint8_t cmd, uint8_t* f, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    f[0] = cmd;
    f[1] = 0;
    f[2] = cmd;
    return transact(f, 3, 32, 0, 0, status);
}

u8 M8266WIFI_SPI_Set_Opmode(u8 op_mode, u8 saved, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[32] = { 0x15, 2, op_mode, saved };
    if (!send(f, 5, status))
        return 0;

    delay_ms(20);
    return f[2];
}

u8 M8266WIFI_SPI_Set_Tx_Max_Power(u8 tx_max_power, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[32] = { 0x2a, 1, tx_max_power > 82 ? 82 : tx_max_power };
    seal(f);
    if (!transact(f, 4, 32, 1000, 0, status))
        return 0;

    return f[2];
}

u8 M8266WIFI_SPI_Get_STA_Connection_Status(u8* connection_status, u16* status)
{
    uint8_t f[32];
    // Older modules report 2 (wrong password) for a while during a connect; poll
    // through it for up to 5 s.
    for (uint8_t tries = 51;;) {
        if (!query(0x12, f, status))
            return 0;

        if (ver_cmp("1.2.0-A") >= 0 || f[2] != 2 || --tries == 0)
            break;

        delay_ms(100);
    }
    if (connection_status)
        *connection_status = f[2];

    return 1;
}

u8 M8266WIFI_SPI_Get_STA_IP_Addr(char* sta_ip, u16* status)
{
    uint8_t f[32];
    if (!query(0x11, f, status))
        return 0;

    for (int i = 0; i < 15 && (sta_ip[i] = f[2 + i]); i++) {}
    sta_ip[15] = 0;
    return 1;
}

u8 M8266WIFI_SPI_STA_DisConnect_Ap(u16* status)
{
    if (module_older("1.1.6-N", status))
        return 0;

    uint8_t f[32];
    if (!query(0x20, f, status))
        return 0;

    if (f[2])
        return 1;

    fail(status, 0x4e);
    return 0;
}

// Reads one parameter: request cmd, 1, type; reply cmd, len, ok, type, value[len - 2].
static u8 param_get(uint8_t cmd, uint8_t type, u8* param, u8* len, uint8_t cap, u16* status)
{
    if (!begin("1.1.6-4", status))
        return 0;

    uint8_t f[32] = { cmd, 1, type };
    if (!send(f, 4, status))
        return 0;

    if (!f[2]) {
        fail(status, 0x80);
        return 0;
    }
    // a reply shorter than its ok and type bytes carries no value
    uint8_t n = f[1] >= 2 ? f[1] - 2 : 0;
    for (uint16_t i = 0; i < n && i < cap; i++)
        param[i] = f[4 + i];

    if (len)
        *len = n;

    return 1;
}

// Values longer than one frame come back in pieces: type 0 continues as 16, type 1 as
// 32 and 33, each piece 27 bytes on.
static u8 param_get_long(uint8_t cmd, uint8_t type, u8* param, u8* len, uint8_t cap,
                         u16* status)
{
    if (type > 1)
        return param_get(cmd, type, param, len, cap, status);

    static const uint8_t more[2][2] = { { 16, 0 }, { 32, 33 } };
    u8 n = 0;
    for (int part = 0;; part++) {
        uint8_t t = part ? more[type][part - 1] : type;
        uint8_t room = cap > 27 * part ? cap - 27 * part : 0;
        if (!param_get(cmd, t, param + 27 * part, &n, room, status))
            return 0;

        if (n <= 26 || part == 2 || (type == 0 && part == 1)) {
            if (len)
                *len = n + 27 * part;

            return 1;
        }
    }
}

// Writes one parameter: cmd, len, saved, type, value. Station settings (cmd 40) give
// the module 10 ms before the reply is polled.
static u8 param_set(uint8_t cmd, uint8_t type, const u8* param, u8 len, u8 saved, u16* status)
{
    if (!begin("1.1.6-4", status))
        return 0;

    uint8_t max = ver_cmp("1.2.0-A") >= 0 ? 27 : 26;
    uint8_t n = len < max ? len : max;
    uint8_t f[40] = { cmd, n + 2, saved, type };
    if (n)
        memcpy(&f[4], param, n);

    seal(f);
    uint16_t pre = cmd == 40 && type != 0xff ? 10 : 0;
    if (!transact(f, n + 5, 32, pre, 2201, status))
        return 0;

    if (!f[2]) {
        fail(status, 0x80);
        return 0;
    }
    delay_ms(20);
    return 1;
}

// SSID (32 bytes) and password (64 bytes) go out in numbered pieces of up to 25 bytes;
// only the last piece carries the saved flag.
static u8 param_set_chunked(uint8_t cmd, uint8_t type, const u8* value, uint8_t len,
                            uint8_t total, u8 saved, u16* status)
{
    uint8_t buf[27];
    for (uint8_t piece = 0, off = 0; off < total; piece++, off += 25) {
        uint8_t n = total - off < 25 ? total - off : 25;
        buf[0] = 0x00;
        buf[1] = 0xc0 | piece;
        memset(&buf[2], 0, n);
        if (off < len)
            memcpy(&buf[2], value + off, len - off < n ? len - off : n);

        u8 last = off + n == total;
        u8 ok = param_set(cmd, type, buf, n + 2, last ? saved : 0, status);
        if (!ok || last)
            return ok;
    }
    return 0;
}

static u8 param_query(uint8_t cmd, u8 long_values, uint8_t type, u8* param, u8* param_len,
                      u16* status)
{
    uint8_t cap = type == 0 ? 32 : type == 1 ? 64 : type == STA_PARAM_TYPE_HOSTNAME ? 29 : 16;
    u8 n = 0;
    u8 ok = long_values ? param_get_long(cmd, type, param, &n, cap, status)
                        : param_get(cmd, type, param, &n, cap, status);
    if (!ok)
        return 0;

    if (n > cap)
        n = cap;

    if (n < cap)
        param[n] = 0;

    if (param_len)
        *param_len = n;

    return 1;
}

u8 M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE param_type, u8* param, u8* param_len,
                                 u16* status)
{
    return param_query(29, ver_cmp("1.1.6-7") > 0, param_type, param, param_len, status);
}

u8 M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE param_type, u8* param, u8* param_len,
                                u16* status)
{
    return param_query(39, ver_cmp("1.2.0-A") >= 0, param_type, param, param_len, status);
}

u8 M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE param_type, u8* param, u8 param_len, u8 saved,
                                 u16* status)
{
    if (ver_cmp("1.2.0-A") < 0 || param_type > 1)
        return param_set(38, param_type, param, param_len, saved, status);

    uint8_t total = param_type ? 64 : 32;
    return param_set_chunked(38, param_type, param, param_len < total ? param_len : total, total,
                             saved, status);
}

// Maps a terminal connection state to its error code; 0 means keep polling.
static uint8_t connect_error(u8 cs)
{
    switch (cs) {
    case 2: return 0x4b;        // wrong password
    case 3: return 0x4a;        // AP not found
    case 4: return 0x4d;        // connect failed
    case 255: return 0x4c;
    default: return 0;
    }
}

static u8 connect_legacy(u8* ssid, u8* password, u8 saved, u8 timeout_in_s, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[32] = { 0x22, 29 };
    for (int i = 0; i < 13 && ssid[i]; i++)
        f[2 + i] = ssid[i];

    for (int i = 0; i < 13 && password[i]; i++)
        f[16 + i] = password[i];

    f[30] = saved;
    if (!send(f, 32, status))
        return 0;

    if (!f[2]) {
        fail(status, 0x80);
        return 0;
    }
    for (u32 i = 0; i != (u32)timeout_in_s * 5; i++) {
        delay_ms(200);
        u8 cs;
        if (M8266WIFI_SPI_Get_STA_Connection_Status(&cs, status) != 1)
            continue;

        if (cs == 5)
            return 1;

        if (connect_error(cs)) {
            fail(status, connect_error(cs));
            return 0;
        }
    }
    fail(status, 0x32);
    return 0;
}

// Sends a station string as 27-byte pieces, one parameter type per piece: the module places
// each type at a fixed offset
static u8 sta_set_string(const uint8_t* types, const u8* v, uint8_t max, u16* status)
{
    uint8_t len = 0;
    while (len < max && v[len])
        len++;

    for (uint8_t part = 0, off = 0;; part++, off += 27) {
        uint8_t n = len - off > 27 ? 27 : len - off;
        if (!param_set(40, types[part], v + off, n, 0, status))
            return 0;

        if (len - off <= 27)
            return 1;
    }
}

u8 M8266WIFI_SPI_STA_Connect_Ap(u8 ssid[32], u8 password[64], u8 saved, u8 timeout_in_s,
                                u16* status)
{
    static const uint8_t ssid_types[] = { STA_PARAM_TYPE_SSID, 16 };
    static const uint8_t pass_types[] = { STA_PARAM_TYPE_PASSWORD, 32, 33 };
    if (ver_cmp("1.1.6-7") <= 0)
        return connect_legacy(ssid, password, saved, timeout_in_s, status);

    if (ver_cmp("1.1.8-8") <= 0 && ver_cmp("1.1.6-N") >= 0) {
        M8266WIFI_SPI_STA_DisConnect_Ap(status);
        M8266HostIf_delay_us(250);
    }
    if (ssid) {
        if (!sta_set_string(ssid_types, ssid, 32, status))
            return 0;

        if (password && !sta_set_string(pass_types, password, 64, status))
            return 0;
    }
    // Type 0xff applies the settings and starts connecting.
    if (!param_set(40, 0xff, NULL, 0, saved, status))
        return 0;

    for (int i = 0; i < timeout_in_s * 20; i++) {
        delay_ms(50);
        u8 cs;
        if (M8266WIFI_SPI_Get_STA_Connection_Status(&cs, status) != 1)
            continue;

        if (cs == 5)
            return 1;

        // "AP not found" is only believed after the first 1.25 s.
        if (connect_error(cs) && (cs != 3 || i > 24)) {
            fail(status, connect_error(cs));
            return 0;
        }
    }
    fail(status, 0x32);
    return 0;
}

static void put_be32(uint8_t* p, u32 v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static u8 scan_error(uint8_t code, u16* status)
{
    switch (code) {
    case 0xfd: fail(status, 0x27); break;
    case 0xfe: fail(status, 0x26); break;     // scan still running
    case 0xff: fail(status, 0x25); break;
    default: fail(status, 0x29); break;
    }
    return 0;
}

// Older modules hand out one signal per frame until they answer 0xfc.
static u8 scan_fetch_legacy(struct ScannedSigs* sigs, u8 max, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.6-3", status))
        return 0;

    for (uint16_t i = 0;; i++) {
        if (i >= max)
            return i;

        if (!ready(status))
            return 0;

        uint8_t f[32] = { 0x1e, 2, i, max - 1 > i ? 0 : 1 };
        seal(f);
        if (!transact(f, 5, 32, 0, 0, status))
            return 0;

        if (f[2] == 0xfc)
            return i;

        if (f[2])
            return scan_error(f[2], status);

        if (!sigs)
            continue;

        uint8_t* e = (uint8_t*)&sigs[i];
        memcpy(e, &f[3], 25);
        memset(e + 25, 0, 7);
        e[32] = f[28];
        e[33] = f[29];
        e[34] = f[30];
    }
}

// Newer modules report the count first, then each signal in two parts.
static u8 scan_fetch(struct ScannedSigs* sigs, u8 max, u16* status)
{
    if (ver_cmp("1.2.0-A") < 0)
        return scan_fetch_legacy(sigs, max, status);

    if (!ready(status))
        return 0;

    uint8_t f[32] = { 0x48, 3, 0xff, 0, 0 };
    if (!send(f, 6, status))
        return 0;

    if (f[2])
        return scan_error(f[2], status);

    uint8_t count = f[3];
    uint16_t i = 0;
    for (; i < max && i < count; i++) {
        uint8_t last = max - 1 == i || count - 1 == i;
        uint8_t name[32], len = 0;
        for (uint8_t part = 0; part < 2; part++) {
            if (!ready(status))
                return 0;

            uint8_t q[32] = { 0x48, 3, i, part ? last : 0, part };
            seal(q);
            if (!transact(q, 6, 32, 0, 0, status))
                return 0;

            if (q[2])
                return scan_error(0, status);

            if (part == 0) {
                len = q[3] > 32 ? 32 : q[3];
                memcpy(name, &q[4], 27);
                continue;
            }
            memcpy(&name[27], &q[3], 5);
            if (sigs) {
                // A 32-byte name's terminator lands on channel, which is written after.
                uint8_t* e = (uint8_t*)&sigs[i];
                memcpy(e, name, len);
                e[len] = 0;
                e[32] = q[8];
                e[33] = q[9];
                e[34] = q[10];
            }
        }
    }
    return i;
}

u8 M8266WIFI_SPI_STA_Fetch_Last_Scanned_Signals(struct ScannedSigs scanned_signals[],
                                                u8 max_signals, u16* status)
{
    return scan_fetch(scanned_signals, max_signals, status);
}

// Older modules take the request in one frame, newer ones in three (SSID filter,
// BSSID filter, timing); only the newer ones get the minimum scan time.
static u8 scan_request(u8 channel, u8 hidden, u8 passive, u32 tmax, u32 tmin, u16* status)
{
    uint8_t f[32] = { 0x21, 11, channel, hidden, passive == 1 };
    if (ver_cmp("1.2.0-A") < 0) {
        put_be32(&f[5], tmax);
        put_be32(&f[9], tmax);
        seal(f);
        if (!transact(f, 14, 32, 0, 0, status))
            return 0;
    } else {
        for (uint8_t part = 0; part < 3; part++) {
            if (part && !ready(status))
                return 0;

            memset(f, 0, sizeof f);
            f[0] = 0x21;
            f[1] = part == 0 ? 29 : part == 1 ? 25 : 15;
            f[2] = channel;
            f[3] = 0xc0 | part;
            if (part == 2) {
                f[4] = passive == 1;
                put_be32(&f[5], tmax);
                put_be32(&f[9], tmin);
                f[13] = hidden;
            }
            seal(f);
            if (!transact(f, f[1] + 3, 32, 0, 0, status))
                return 0;

            if (f[2] && part < 2)
                break;
        }
    }
    if (f[2]) {
        if (status) {
            *status = (u16)(reg_read() << 8);
            M8266HostIf_delay_us(1);
            *status |= 0x3e;
        }
        return 0;
    }
    return 1;
}

u8 M8266WIFI_SPI_STA_ScanSignals(struct ScannedSigs scanned_signals[], u8 max_signals,
                                 u8 channel, u8 show_hidden, u8 passive_not_active_scan,
                                 u32 channel_scan_time_ms_max, u32 channel_scan_time_ms_min,
                                 u8 timeout_in_s, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.6-3", status))
        return 0;

    u8 cs = 0;
    if (!M8266WIFI_SPI_Get_STA_Connection_Status(&cs, status))
        return 0;

    // A station mid-connect can't scan; drop it now and reconnect afterwards.
    u8 interrupted = cs != 0 && cs != 5 && cs != 255;
    if (interrupted)
        M8266WIFI_SPI_STA_DisConnect_Ap(NULL);

    u8 found = 0;
    if (ready(status)
        && scan_request(channel, show_hidden == 1, passive_not_active_scan,
                        channel_scan_time_ms_max, channel_scan_time_ms_min, status)) {
        delay_ms(10);
        for (uint16_t left = timeout_in_s * 100;; left--) {
            u16 st = 0;
            found = scan_fetch(scanned_signals, max_signals, &st);
            if (status)
                *status = st;

            if (found || (st & 0xff) != 0x26)
                break;

            if (!left) {
                fail(status, 0x28);
                break;
            }
            delay_ms(10);
        }
    }
    if (interrupted) {
        u16 reconnect;
        param_set(40, 0xff, NULL, 0, 0, &reconnect);
    }

    return found;
}

// Resolves a host name (or dotted address) to a dotted address. Names longer than 27
// characters go out in 24-byte pieces to newer modules.
static u8 resolve_host(char ip[16], const char* name, u8 timeout_in_s, u16* status)
{
    static const char broadcast[16] = "255.255.255.255";
    if (!name) {
        fail(status, 0x4f);
        return 0;
    }
    uint16_t len = 0;
    while (len < 256 && name[len])
        len++;

    if (len == 0 || len == 256) {
        fail(status, 0x4f);
        return 0;
    }
    uint16_t timeout_ms = (timeout_in_s > 30 ? 30 : timeout_in_s) * 1000;
    uint8_t f[32];
    u8 long_name = 0;

    if (len > 27 && !module_older("1.1.8-R", status)) {
        if (status)
            *status = 0;

        uint8_t got = 0;
        for (uint16_t off = 0; off < len;) {
            uint8_t n = len - off > 23 ? 24 : len - off;
            if (!begin(NULL, status))
                return 0;

            f[0] = 54;
            f[1] = n + 5;
            f[2] = len >> 8;
            f[3] = len;
            f[4] = off >> 8;
            f[5] = off;
            f[6] = n;
            memcpy(&f[7], name + off, n);
            seal(f);
            if (!transact(f, f[1] + 3, 32, 0, timeout_ms, status))
                return 0;

            if (f[2]) {
                long_name = 1;
                goto refused;
            }
            got = f[1];
            off += n;
        }
        uint8_t i = 0;
        for (; i < got && i < 15 && (ip[i] = f[3 + i]); i++) {}
        ip[i] = 0;
        return 1;
    }

    if (status)
        *status = 0;

    if (module_older("1.1.6-3", status))
        return 0;

    if (!strcmp(name, broadcast)) {
        memcpy(ip, broadcast, 16);
        return 1;
    }
    if (!ready(status))
        return 0;

    f[0] = 37;
    f[2] = timeout_in_s;
    uint8_t n = 0;
    while (n < 27 && (f[3 + n] = name[n]))
        n++;

    f[3 + n] = 0;
    f[1] = n + 2;
    seal(f);
    if (!transact(f, f[1] + 3, 32, 0, timeout_ms, status))
        return 0;

    if (f[2])
        goto refused;

    uint8_t i = 0;
    for (; i < f[1] - 2 && i < 15 && (ip[i] = f[3 + i]); i++) {}
    ip[i] = 0;
    return 1;

refused:
    if (status) {
        *status = (u16)(reg_read() << 8);
        switch (f[2]) {
        case 156: *status |= long_name ? 0x4f : 0x3d; break;
        case 250:
        case 251: *status |= 0x5f; break;
        case 252: *status |= 0x5e; break;
        case 253: *status |= 0x3c; break;
        case 254: *status |= 0x3b; break;
        case 255: *status |= 0x3a; break;
        default: *status |= 0x3d; break;
        }
    }
    return 0;
}

// Link info: type (0 UDP, 1 TCP client, 2 TCP server) and state.
static u8 query_link(u8 link_no, u8* type, u8* state, u16* status)
{
    if (!begin(NULL, status))
        return 0;

    uint8_t f[32] = { 0x14, 1, link_no };
    if (!send(f, 4, status))
        return 0;

    if (f[3]) {
        if (type)
            *type = 0;

        if (state)
            *state = 0;

        return 1;
    }
    if (type)
        *type = f[4];

    if (state)
        *state = f[5];

    return 1;
}

static u8 link_is(u8 link_no, u8 type, u16* status)
{
    u8 t = 0xff;
    if (!query_link(link_no, &t, NULL, status))
        return 0;

    if (t == type)
        return 1;

    fail(status, type + 10);
    return 0;
}

// Client idx on a TCP server link: reply holds count, ip, port and state.
static u8 query_client(u8 link_no, u8 idx, u8* count, u8* state, u8 ip[4], u16* port,
                       u16* status)
{
    if (count)
        *count = 0;

    if (!begin(NULL, status))
        return 0;

    uint8_t f[32] = { 0x2f, 2, link_no, idx };
    if (!send(f, 5, status))
        return 0;

    if (f[4]) {
        if (status) {
            *status = (u16)(reg_read() << 8);
            switch (f[4]) {
            case 255: *status |= 0x5a; break;
            case 254: *status |= 0x5b; break;   // no clients
            case 253: *status |= 0x5c; break;
            case 252: *status |= 0x5d; break;
            default: *status |= 0x35; break;
            }
        }
        return 0;
    }
    if (count)
        *count = f[5];

    if (ip)
        memcpy(ip, &f[6], 4);

    if (port)
        *port = f[10] << 8 | f[11];

    if (state)
        *state = f[12];

    return 1;
}

// Closes a link, or with keep_link only the server's client at ip:port.
static u8 close_link(u8 keep_link, u8 link_no, const u8* ip, u16 port, u16* status)
{
    if (status)
        *status = 0;

    u8 type, state;
    if (!query_link(link_no, &type, &state, status))
        return 0;

    uint8_t err = 0;
    if (type > 2 || (type && !state)) {
        err = 0x54;
    } else if (keep_link) {
        if (type == 0)
            err = 0x57;
        else if (type == 1 && state == 6)
            err = 0x57;
        else if (type == 2) {
            u8 clients = 0;
            if (!query_client(link_no, 0, &clients, NULL, NULL, NULL, status))
                return 0;

            if (!clients)
                err = 0x57;
        }
    }
    if (err) {
        fail(status, err);
        return 0;
    }
    if (!ready(status))
        return 0;

    uint8_t f[32] = { keep_link ? 26 : 25, 1, link_no };
    if (ip) {
        memcpy(&f[3], ip, 4);
        f[7] = port >> 8;
        f[8] = port;
        f[1] = 7;
    }
    if (!send(f, f[1] + 3, status))
        return 0;

    if (f[2] && f[2] != 0xf4) {
        if (status)
            *status = (u16)(reg_read() << 8 | (uint32_t)(128 - f[2]));

        return 0;
    }
    if (type)
        delay_ms(10);

    return 1;
}

u8 M8266WIFI_SPI_Delete_Connection(u8 link_no, u16* status)
{
    return close_link(0, link_no, NULL, 0, status);
}

u8 M8266WIFI_SPI_Setup_Connection(u8 tcp_udp, u16 local_port, char* remote_addr,
                                  u16 remote_port, u8 link_no, u8 timeout_in_s, u16* status)
{
    if (status)
        *status = 0;

    // refused before the frame: the module would open the link
    u8 type = tcp_udp & 15;
    if (type > 2 || (ver_cmp("1.1.8-X") < 0 && tcp_udp > 2) || (tcp_udp > 15 && type != 1)) {
        uint8_t s = reg_read();
        if (status)
            *status = (u16)(s << 8 | 0x77);

        return 0;
    }
    if (ver_cmp("1.1.6-N") >= 0) {
        u16 st = 0;
        if (!M8266WIFI_SPI_Delete_Connection(link_no, &st)
            && ((st & 0xff) == 0x55 || (st & 0xff) == 0x56)) {
            if (status)
                *status = st;

            return 0;
        }
    }
    char host[16];
    if (ver_cmp("1.1.6-3") >= 0) {
        if (!resolve_host(host, remote_addr, timeout_in_s, status))
            return 0;
    } else {
        uint8_t i = 0;
        for (; i < 15 && (host[i] = remote_addr[i]); i++) {}
        host[i] = 0;
    }
    if (!ready(status))
        return 0;

    uint8_t f[32] = { 0x18, 22, tcp_udp, local_port >> 8, local_port };
    for (int i = 0; i < 16 && host[i]; i++)
        f[5 + i] = host[i];

    f[21] = remote_port >> 8;
    f[22] = remote_port;
    f[23] = link_no;
    seal(f);
    if (!transact(f, 25, 32, 5, 0, status))
        return 0;

    if (f[2]) {
        if (status)
            *status = (u16)(reg_read() << 8 | (uint32_t)(128 - f[2]));

        return 0;
    }
    if (type != 1)
        return 1;

    // TCP client: wait for the connect to settle.
    for (u32 i = 0; i != (u32)timeout_in_s * 5; i++) {
        delay_ms(200);
        u8 state = 0;
        if (!query_link(link_no, NULL, &state, status))
            continue;

        if (state >= 2 && state <= 5)
            return 1;

        if (state == 0 || state == 6) {
            fail(status, state ? 0x39 : 0x33);
            return 0;
        }
    }
    fail(status, 0x34);
    return 0;
}

u8 M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(u8 link_no, u16 timeout_in_s, u16* status)
{
    if (status)
        *status = 0;

    if (!link_is(link_no, 2, status) || !ready(status))
        return 0;

    uint8_t f[32] = { 0x1b, 3, link_no, timeout_in_s >> 8, timeout_in_s };
    if (!send(f, 6, status))
        return 0;

    if (f[2]) {
        fail(status, 0x3f);
        return 0;
    }
    return 1;
}

// A link's TCP options: query cmd 0x2c {link, opt} -> {code, n, value[n]}, config cmd 0x2d
// {link, opt, n, value[n]} -> {code}. Option 0 is a flag byte, 3 the window count.
static u8 tcp_opt_query(u8 link, u8 opt, u8* value, u8 cap, u16* status)
{
    if (!begin("1.1.6-F", status))
        return 0;

    uint8_t f[32] = { 0x2c, 2, link, opt };
    if (!send(f, 5, status))
        return 0;

    if (f[2]) {
        fail(status, 0x40);
        return 0;
    }
    memcpy(value, &f[4], f[3] < cap ? f[3] : cap);
    return 1;
}

static u8 tcp_opt_config(u8 link, u8 opt, const u8* value, u8 n, u16* status)
{
    if (!begin("1.1.6-F", status))
        return 0;

    uint8_t f[32] = { 0x2d, (uint8_t)(n + 3), link, opt, n };
    memcpy(&f[5], value, n);
    if (!send(f, n + 6, status))
        return 0;

    if (f[2]) {
        fail(status, 0x41);
        return 0;
    }
    return 1;
}

u8 M8266WIFI_SPI_Query_Tcp_Window_num(u8 link_no, u8* tcp_wnd_num, u16* status)
{
    return tcp_opt_query(link_no, 3, tcp_wnd_num, 1, status);
}

u8 M8266WIFI_SPI_Config_Tcp_Window_num(u8 link_no, u8 tcp_wnd_num, u16* status)
{
    return tcp_opt_config(link_no, 3, &tcp_wnd_num, 1, status);
}

u8 M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(u8 server_link_no, u8 max_allowed,
                                                            u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status) || !link_is(server_link_no, 2, status))
        return 0;

    return tcp_opt_config(server_link_no, 7, &max_allowed, 1, status);
}

u8 M8266WIFI_SPI_List_Clients_On_A_TCP_Server(u8 server_link_no, u8* clients,
                                              ClientInfo RemoteClients[], u16* status)
{
    if (!link_is(server_link_no, 2, status))
        return 0;

    if (clients)
        *clients = 0;

    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status))
        return 0;

    u8 count = 0;
    u16 st = 0;
    ClientInfo* c = &RemoteClients[0];
    if (!query_client(server_link_no, 0, &count, &c->connection_state, c->remote_ip,
                      &c->remote_port, &st)) {
        // "No clients" is an empty list, not an error.
        u8 none = (st & 0xff) == 0x5b || (st & 0xff) == 0x5c;
        if (clients)
            *clients = 0;

        if (status)
            *status = none ? 0 : st;

        return none;
    }
    if (count > M8266WIFI_MAX_CLIENTS)
        count = M8266WIFI_MAX_CLIENTS;

    if (clients)
        *clients = count;

    for (u8 i = 1; i < count; i++) {
        c = &RemoteClients[i];
        u8 listed;
        if (!query_client(server_link_no, i, &listed, &c->connection_state, c->remote_ip,
                          &c->remote_port, status)) {
            if (clients)
                *clients = i;

            return 1;
        }
    }
    return 1;
}

u8 M8266WIFI_SPI_Disconnect_TcpClient(u8 link_no, ClientInfo* client_info, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status) || !link_is(link_no, 2, status))
        return 0;

    if (!client_info)
        return close_link(1, link_no, NULL, 0, status);

    return close_link(1, link_no, client_info->remote_ip, client_info->remote_port, status);
}

// Raises the request flag for a payload transfer and drops the flags outside keep_mask,
// once the register stops changing.
static void data_mode(uint8_t keep_mask)
{
    uint8_t s = reg_read();
    if (((s & keep_mask) | ST_REQ) == s)
        return;

    for (uint16_t n = 1000; n; n--) {
        uint8_t v = reg_read();
        if (v == s)
            break;

        M8266HostIf_delay_us(1);
        s = v;
    }
    uint8_t w = (s & keep_mask) | ST_REQ;
    if (w != s) {
        reg_write(w);
        M8266HostIf_delay_us(10);
    }
}

// Header frame (cmd 0) with length and destination, then the payload in raw 32-byte
// buffer writes, from data, or else each piece from give() just before it is written. Returns
// the bytes handed over.
static u16 send_to(const u8* data, M8266_TxGive give, void* ctx, u16 len, u8 link_no,
                   const char* addr, u16 port, u16* status)
{
    if (status)
        *status = 0;

    if (!len)
        return 0;

    char host[16];
    if (ver_cmp("1.1.6-3") >= 0) {
        if (!resolve_host(host, addr, 20, status))
            return 0;
    } else {
        uint8_t i = 0;
        for (; i < 15 && (host[i] = addr[i]); i++) {}
        host[i] = 0;
    }

    u16 sent = 0;
    if (!wait_ready(status))
        goto out;

    data_mode((uint8_t)~(ST_DATA_END | ST_REPLY));
    uint8_t f[32] = { 0x00, 22, link_no, len >> 8, len, 1 };
    for (int i = 0; i < 16 && host[i]; i++)
        f[6 + i] = host[i];

    f[22] = port >> 8;
    f[23] = port;
    seal(f);
    if (!transact(f, 25, 4, 0, 0, status))
        goto out;

    if (f[2]) {
        static const uint8_t codes[10] = { 0x1d, 0x1c, 0x1b, 0x1a, 0x19,
                                           0x18, 0x15, 0x14, 0x13, 0x12 };
        if (status) {
            *status = (u16)(reg_read() << 8);
            *status |= f[2] >= 246 ? codes[f[2] - 246] : 0x1f;
        }
        if (f[2] == 255)
            M8266HostIf_delay_us(200);

        goto out;
    }
    M8266HostIf_delay_us(11);
    while (sent < len) {
        uint8_t n = len - sent > 32 ? 32 : len - sent;
        if (data) {
            buf_write(data + sent, n);
        } else {
            u8 piece[32];
            give(ctx, sent, piece, n);
            buf_write(piece, n);
        }
        M8266HostIf_delay_us(reread_after_write ? 8 : 11);
        if (!wait_picked_up(status))
            goto out;

        sent += n;
    }

out:
    flag_set(ST_DATA_END, 1, 10);
    M8266HostIf_delay_us(2);
    for (int n = 15;; n--) {
        if (!n) {
            flag_set(ST_DATA_END, 1, 10);
            break;
        }
        uint8_t s = reg_read();
        if (s != 0xff && (s & ST_DATA_END))
            break;

        M8266HostIf_delay_us(1);
    }
    // Short packets get a pause scaled to how short they are.
    if (len > sent || len > 256)
        return sent;

    if (len > 249)
        M8266HostIf_delay_us(10);
    else {
        M8266HostIf_delay_us(20);
        M8266HostIf_delay_us(255 - len);
    }
    return sent;
}

u16 M8266WIFI_SPI_Send_Udp_Data(u8 Data[], u16 Data_len, u8 link_no, char* udp_dest_addr,
                                u16 udp_dest_port, u16* status)
{
    if (module_older("1.1.6-F", status))
        return 0;

    return send_to(Data, 0, 0, Data_len, link_no, udp_dest_addr, udp_dest_port, status);
}

u16 M8266WIFI_SPI_Send_Data_to_TcpClient(u8 Data[], u16 Data_len, u8 server_link_no,
                                         char* tcp_client_dest_addr,
                                         u16 tcp_client_dest_port, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status))
        return 0;

    return send_to(Data, 0, 0, Data_len, server_link_no, tcp_client_dest_addr,
                   tcp_client_dest_port, status);
}

u16 M8266WIFI_SPI_Send_to_TcpClient_from(M8266_TxGive give, void* ctx, u16 Data_len,
                                         u8 server_link_no, char* tcp_client_dest_addr,
                                         u16 tcp_client_dest_port, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status))
        return 0;

    return send_to(0, give, ctx, Data_len, server_link_no, tcp_client_dest_addr,
                   tcp_client_dest_port, status);
}

// Asks for up to max_len bytes, then pulls them chunk by chunk: the host raises bit 5,
// the module drops it once the next chunk is in the buffer, and bit 6 marks the last. The
// reply's head names the sender before any data moves, so where() may pick the memory, or
// none: then each chunk goes to take() as it is read.
static u16 recv(u8* Data, M8266_RxDest where, M8266_RxTake take, void* ctx, u16 max_len,
                uint16_t max_wait_in_ms, u8* link_no, u8 remote_ip[4], u16* remote_port,
                u16* status)
{
    u8 piece[32];
    u8 link = link_no ? *link_no : 0;
    u16 got = 0, err = 0;
    if (status)
        *status = 0;

    uint32_t t0 = M8266HostIf_now_us();
    for (;;) {
        if (expired(t0, 1000u * max_wait_in_ms)) {
            fail(status, 0x10);
            goto out;
        }

        if (M8266WIFI_SPI_Has_DataReceived())
            break;

        poll_pause(t0);
    }
    data_mode((uint8_t)~(ST_CHUNK_REQ | ST_REPLY));
    uint8_t f[32] = { 0x01, 2, max_len >> 8, max_len };
    seal(f);
    if (!transact(f, 5, 19, 0, 0, status))
        goto out;

    u16 n = max_len;
    if (f[2] && status)
        *status = (u16)(reg_read() << 8);

    if (f[2] == 0xff) {
        if (status)
            *status |= 0x22;

        M8266HostIf_delay_us(200);
        goto out;
    }
    if (f[2] && f[2] < 0xfd) {
        if (status)
            *status |= 0x2f;

        goto out;
    }
    if (f[2] == 0xfe && status)
        *status |= 0x23;

    u16 avail = f[4] << 8 | f[5];
    if (f[2] != 0xfd && max_len >= avail)
        n = avail;
    else if (status && !(*status & 0xff))
        *status |= 0x24;        // more waiting than fits

    link = f[3];
    u8 ip[4] = { 0 };
    u16 port = 0;
    if (ver_cmp("1.1.6-5") > 0) {
        port = f[11] << 8 | f[10];
        memcpy(ip, &f[14], 4);
    }
    if (remote_port)
        *remote_port = port;

    if (remote_ip)
        memcpy(remote_ip, ip, 4);

    if (where)
        Data = where(ctx, link, ip, port, n);

    if (!reread_after_write)
        M8266HostIf_delay_us(11);

    u8 retried = 0;
    while (got < n) {
        flag_set(ST_CHUNK_REQ, 0, reread_after_write ? 0 : 5);
        uint8_t s = 0;
        u8 loaded = 0;
        uint32_t t1 = M8266HostIf_now_us();
        for (;;) {
            s = reg_read();
            loaded = s && !(s & ST_CHUNK_REQ);
            if (loaded || expired(t1, POLL_TIMEOUT_US))
                break;

            poll_pause(t1);
        }
        if (loaded) {
            err = 0;
            retried = 0;
        } else {
            err = (u16)(s << 8 | 0x20);
            if (retried)
                break;

            flag_clear(ST_CHUNK_REQ, 10);
            M8266HostIf_delay_us(5);
            retried = 1;
            continue;
        }
        uint8_t chunk = n - got > 32 ? 32 : n - got;
        if (Data) {
            buf_read(Data + got, chunk);
        } else {
            buf_read(piece, chunk);
            take(ctx, piece, chunk);
        }
        got += chunk;
        if (got >= n)
            M8266HostIf_delay_us(1);

        s = reg_read();
        if (s == 0xff || !(s & ST_LAST_CHUNK))
            continue;

        // Last chunk flagged: pick up a short remainder if the module already has it.
        if (got >= n || n - got > 32)
            break;

        s = reg_read();
        if (s && !(s & ST_CHUNK_REQ)) {
            if (Data) {
                buf_read(Data + got, n - got);
            } else {
                buf_read(piece, n - got);
                take(ctx, piece, n - got);
            }
            got = n;
        }
        break;
    }

out:
    if (link_no)
        *link_no = link;

    if (status) {
        uint8_t code = *status & 0xff;
        if (err)
            *status = err;
        else if (code == 0 || (code >= 0x22 && code <= 0x24))
            *status = code;
    }
    return got;
}

u16 M8266WIFI_SPI_RecvData_ex(u8 Data[], u16 max_len, uint16_t max_wait_in_ms, u8* link_no,
                              u8 remote_ip[4], u16* remote_port, u16* status)
{
    return recv(Data, 0, 0, 0, max_len, max_wait_in_ms, link_no, remote_ip, remote_port,
                status);
}

u16 M8266WIFI_SPI_RecvData_to(M8266_RxDest where, M8266_RxTake take, void* ctx, u16 max_len,
                              uint16_t max_wait_in_ms, u8* link_no, u8 remote_ip[4],
                              u16* remote_port, u16* status)
{
    return recv(0, where, take, ctx, max_len, max_wait_in_ms, link_no, remote_ip, remote_port,
                status);
}

// The name the station gives the router with DHCP: cmd 0x1c reads it (at most 28
// characters), cmd 0x24 sets it (the name without its terminator, at most 29).
u8 M8266WIFI_SPI_Get_STA_Hostname(char hostname[28 + 1], u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.5-G", status))
        return 0;

    uint8_t f[32];
    if (!query(0x1c, f, status))
        return 0;

    if (!f[2]) {
        fail(status, 0x37);
        return 0;
    }
    uint8_t n = f[1] == 0 ? 0 : f[1] - 1 > 28 ? 28 : f[1] - 1;
    memcpy(hostname, &f[3], n);
    hostname[n] = 0;
    return 1;
}

u8 M8266WIFI_SPI_Set_STA_Hostname(char hostname[28 + 1], u16* status)
{
    if (!begin("1.1.5-G", status))
        return 0;

    uint8_t f[32] = { 0x24 };
    uint8_t n = 0;
    while (n < 29 && hostname[n]) {
        f[2 + n] = hostname[n];
        n++;
    }
    f[1] = n;
    if (!send(f, n + 3, status))
        return 0;

    if (f[2])
        return 1;

    fail(status, 0x38);
    return 0;
}

// The module's built-in web page: cmd 0x50 starts or stops it on a port (the access code all
// zeros), cmd 0x51 reports it.
u8 M8266WIFI_SPI_Set_WebServer(u8 open_not_shutdown, u16 server_port, u8 saved, u16* status)
{
    if (!begin("1.1.5-9", status))
        return 0;

    uint8_t f[32] = { 0x50, 27, open_not_shutdown, server_port >> 8, server_port, saved };
    if (!send(f, 30, status))
        return 0;

    return f[2] != 0;
}

u8 M8266WIFI_SPI_Query_WebServer(u8* start_on_bootup, u8* current_running, u16* default_port,
                                 u16* current_port, u16* status)
{
    if (status)
        *status = 0;

    if (module_older("1.1.8-1", status))
        return 0;

    uint8_t f[32];
    if (!query(0x51, f, status))
        return 0;

    if (!f[2]) {
        fail(status, 0x80);
        return 0;
    }
    if (current_running)
        *current_running = f[3];

    if (current_port)
        *current_port = f[4] << 8 | f[5];

    if (start_on_bootup)
        *start_on_bootup = f[6];

    if (default_port)
        *default_port = f[7] << 8 | f[8];

    return 1;
}
