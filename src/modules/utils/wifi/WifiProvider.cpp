/*
 * WifiProvider.cpp
 *
 *  Created on: June 10, 2020
 *      Author: Josh
 */

#include "WifiProvider.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "SimpleShell.h"
#include "modules/utils/simpleshell/FileTransfer.h"

#include "M8266HostIf.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ConfigTable.h"
#include "checksumm.h"
#include "Gcode.h"
#include "libs/Logging.h"
#include "libs/StreamOutput.h"
#include "libs/utils.h"
#include "Logging.h"
#include "utils.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"

#include "port_api.h"
#include "InterruptIn.h"

#include "gpio.h"

#include <math.h>

#define WIFI_CONFIG(X) \
    X(bool, enable, "enable", true) \
    X(int, tcp_port, "tcp_port", 2222) \
    X(int, udp_send_port, "udp_send_port", 3333) \
    X(int, udp_recv_port, "udp_recv_port", 4444) \
    X(int, tcp_timeout_s, "tcp_timeout_s", 10) \
    X(str, machine_name, "machine_name", "CARVERA_01001", 32) \
    X(pin, interrupt_pin, "interrupt_pin", "2.11")
CONFIG_STRUCT(WifiConfig, WIFI_CONFIG);
CONFIG_KEYS(wifi_config_keys, WifiConfig, WIFI_CONFIG);
extern WifiProvider wifi_provider;
static void wifi_config_changed(const ConfigTable::Group *, const void *c)
{
    wifi_provider.configure(c);
}
CONFIG_GROUPS(wifi_provider_config_groups,
    CFG_GROUP("wifi", wifi_config_keys, WifiConfig, wifi_config_changed));

// status: the module's status register in the high byte, the error code in the low byte
static void module_error(const char* what, u16 status)
{
    printk("error:wifi %s failed, status 0x%04x\n", what, status);
}

// Ports take effect on the next connection setup, the name on the next broadcast.
void WifiProvider::configure(const void *cfg)
{
    const WifiConfig &c = *(const WifiConfig *)cfg;
    this->tcp_port = c.tcp_port;
    this->udp_send_port = c.udp_send_port;
    this->udp_recv_port = c.udp_recv_port;
    this->tcp_timeout_s = c.tcp_timeout_s;
    strncpy(this->machine_name, c.machine_name, sizeof(this->machine_name));
}


void WifiProvider::on_module_loaded()
{
    udp_link_no = 0;
    tcp_link_no = 1;
    next_available_link_no = 2;
    wifi_init_ok = false;
    has_data_flag = false;
    connection_fail_count = 0;

    const WifiConfig &wifi_config = ConfigTable::config<WifiConfig>(wifi_provider_config_groups);
    configure(&wifi_config);
    if (!wifi_config.enable) {
        // Not needed; free up resources
        return;
    }

	data_callbacks.clear();


    // Initialize WiFi module
    this->init_wifi_module(false);

    // Set up interrupt for WiFi data reception
    Pin* smoothie_pin = new Pin();
    smoothie_pin->from_spec(wifi_config.interrupt_pin);
    smoothie_pin->as_input();
    if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
        PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
        wifi_interrupt_pin = new mbed::InterruptIn(pinname);
        wifi_interrupt_pin->rise(this, &WifiProvider::on_pin_rise);
        NVIC_SetPriority(EINT3_IRQn, 16);
    } else {
        printk("Error: WiFi interrupt pin must be on P0 or P2.\n");
        return;
    }
    delete smoothie_pin;

    // Add this stream to the kernel's stream pool for broadcasting
    THEKERNEL->streams.append_stream(this);

    // Register for events
    ADD_MCODE(m482, 482, IMMEDIATE, WifiProvider::query_sta_param);
    ADD_MCODE(m483, 483, IMMEDIATE, WifiProvider::query_ap_param);
    ADD_MCODE(m489, 489, IMMEDIATE, WifiProvider::report_status);
    beacon.start();
}

void WifiProvider::on_pin_rise()
{
    has_data_flag = true;
}

namespace {
    StaticSemaphore_t module_lock_store;
    SemaphoreHandle_t module_lock = xSemaphoreCreateMutexStatic(&module_lock_store);

    class ModuleLock {
    public:
        ModuleLock() { xSemaphoreTake(module_lock, portMAX_DELAY); }
        ~ModuleLock() { xSemaphoreGive(module_lock); }
        ModuleLock(const ModuleLock&) = delete;
        ModuleLock& operator=(const ModuleLock&) = delete;
    };

    template <typename F>
    auto locked(F f) -> decltype(f())
    {
        ModuleLock lock;
        return f();
    }

    StaticSemaphore_t tx_lock_store;
    SemaphoreHandle_t tx_lock = xSemaphoreCreateMutexStatic(&tx_lock_store);
}

bool WifiProvider::read_chunk(bool dispatch)
{
    if (rx_owner != nullptr) return false;

    u8 link_no = 0xFF;
    u8 remote_ip[4]{};
    u16 remote_port = 0;
    u16 status = 0;
    u16 received;

    {
        ModuleLock lock;
        received = M8266WIFI_SPI_RecvData_ex(rx_buf, WIFI_DATA_MAX_SIZE, WIFI_DATA_TIMEOUT_MS,
                                             &link_no, remote_ip, &remote_port, &status);
    }

    if (link_no == 0xFF || received == 0) return false;   // nothing arrived

    auto it = data_callbacks.find(link_no);
    if (it != data_callbacks.end()) {
        it->second(remote_ip, remote_port, rx_buf, received);
        return received == WIFI_DATA_MAX_SIZE;
    }
    if (link_no != tcp_link_no) return received == WIFI_DATA_MAX_SIZE;  // the udp link: not a console

    Session* s = session_for(remote_ip, remote_port, true);
    if (s == nullptr) return received == WIFI_DATA_MAX_SIZE;   // no room; the module should have refused them
    s->fresh = true;

    if (s->is_transferring()) {
        rx_owner = s;
        rx_len = received;
        return false;
    } else if (dispatch) {
        s->feed(rx_buf, received);
    } else {
        s->queue(rx_buf, received);
    }
    return received == WIFI_DATA_MAX_SIZE;
}

bool WifiProvider::take_held(Session* s, char** buf, int* n)
{
    if (rx_owner != s || rx_len == 0) return false;
    rx_owner = nullptr;
    *buf = (char*)rx_buf;
    *n = rx_len;
    rx_len = 0;
    return true;
}

int WifiProvider::flush_unlocked(bool patient)
{
    if (tx_len == 0 || tx_owner == nullptr) { tx_len = 0; tx_owner = nullptr; return 0; }
    Session* s = tx_owner;
    size_t len = tx_len;
    size_t got = send_to_client(s->who.ip, s->who.port, s->link, (const u8*)tx_buf, len,
                                patient ? WIFI_TX_RETRIES : WIFI_TX_RETRIES_SHORT);
    if (got != len) {
        s->stall = Session::STALL_TICKS;
        if (!patient) {
            memmove(tx_buf, tx_buf + got, len - got);   // waits for the next patient flush
            tx_len = len - got;
            tx_owner = s;
            return (int)got;
        }
    }
    tx_len = 0;
    tx_owner = nullptr;
    return (int)got;
}

int WifiProvider::flush_tx(bool patient)
{
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    int r = flush_unlocked(patient);
    xSemaphoreGive(tx_lock);
    return r;
}

int WifiProvider::stage(Session* s, const uint8_t* data, size_t len)
{
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    int r;
    if (s->is_transferring() || len > WIFI_DATA_MAX_SIZE) {
        flush_unlocked(false);
        r = (int)send_to_client(s->who.ip, s->who.port, s->link, data, len);
    } else if (s->stall != 0) {
        r = 0;
    } else {
        if (tx_len && (tx_owner != s || tx_len + len > WIFI_DATA_MAX_SIZE)) {
            flush_unlocked(false);
            if (tx_owner != s) { tx_len = 0; tx_owner = nullptr; }
        }
        if (tx_len + len <= WIFI_DATA_MAX_SIZE) {
            memcpy(tx_buf + tx_len, data, len);
            tx_len += len;
            tx_owner = s;
            r = tx_len >= WIFI_DATA_MAX_SIZE ? flush_unlocked(false) : (int)len;
        } else {
            r = 0;
        }
    }
    xSemaphoreGive(tx_lock);
    return r;
}

Session* WifiProvider::session_for(const u8 ip[4], u16 port, bool create)
{
    Session* free_slot = nullptr;
    for (Session& s : sessions) {
        if (!s.live()) {
            if (free_slot == nullptr) free_slot = &s;
            continue;
        }
        if (s.is(ip, port)) return &s;
    }
    if (!create || free_slot == nullptr) return nullptr;
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    if (tx_owner == free_slot) { tx_len = 0; tx_owner = nullptr; }
    xSemaphoreGive(tx_lock);
    free_slot->bind(this, tcp_link_no, ip, port);
    return free_slot;
}

void WifiProvider::reap_sessions(const ClientInfo* listed, u8 count)
{
    for (Session& s : sessions) {
        if (!s.live()) continue;
        if (s.stall != 0) s.stall--;
        if (s.fresh) { s.fresh = false; continue; }
        bool known = false;
        for (u8 i = 0; i < count && !known; i++) {
            known = s.is(listed[i].remote_ip, listed[i].remote_port);
        }
        if (!known) {
            printk("wifi: client %u.%u.%u.%u:%u disconnected\n",
                   s.who.ip[0], s.who.ip[1], s.who.ip[2], s.who.ip[3], s.who.port);
            xSemaphoreTake(tx_lock, portMAX_DELAY);
            if (tx_owner == &s) { tx_len = 0; tx_owner = nullptr; }   // the client is gone
            xSemaphoreGive(tx_lock);
            s.release();
        }
    }
}

void WifiProvider::get_broadcast_from_ip_and_netmask(char* broadcast_addr, char* ip_addr, char* netmask)
{
    uint32_t i_ip = ip_to_int(ip_addr);
    uint32_t i_mask = ip_to_int(netmask);
    uint32_t i_broadcast = i_ip | (~i_mask);
    int_to_ip(i_broadcast, broadcast_addr);
}

namespace {
    enum ParamKind : uint8_t { TEXT, NUMBER, MAC, HOSTNAME };
    struct ParamRow { uint8_t type; const char* name; ParamKind kind; };

    const ParamRow sta_params[] = {
        {STA_PARAM_TYPE_SSID,         "ssid",     TEXT},
        {STA_PARAM_TYPE_PASSWORD,     "password", TEXT},
        {STA_PARAM_TYPE_CHANNEL,      "channel",  NUMBER},
        // the hostname has its own command; querying parameter type 3 returns an empty string
        {STA_PARAM_TYPE_HOSTNAME,     "hostname", HOSTNAME},
        {STA_PARAM_TYPE_MAC,          "mac",      MAC},
        {STA_PARAM_TYPE_IP_ADDR,      "ip",       TEXT},
        {STA_PARAM_TYPE_GATEWAY_ADDR, "gateway",  TEXT},
        {STA_PARAM_TYPE_NETMASK_ADDR, "netmask",  TEXT},
    };
    const ParamRow ap_params[] = {
        {AP_PARAM_TYPE_SSID,         "ssid",     TEXT},
        {AP_PARAM_TYPE_PASSWORD,     "password", TEXT},
        {AP_PARAM_TYPE_CHANNEL,      "channel",  NUMBER},
        {AP_PARAM_TYPE_AUTHMODE,     "authmode", NUMBER},
        {AP_PARAM_TYPE_IP_ADDR,      "ip",       TEXT},
        {AP_PARAM_TYPE_GATEWAY_ADDR, "gateway",  TEXT},
        {AP_PARAM_TYPE_NETMASK_ADDR, "netmask",  TEXT},
        {AP_PARAM_TYPE_PHY_MODE,     "phymode",  NUMBER},
    };

    // M482.<n> queries the joined network, M483.<n> the machine's own hotspot
    template <size_t N>
    void query_param(unsigned m, const ParamRow (&rows)[N], bool sta, unsigned sub)
    {
        if (sub >= N) {
            printk("error:M%u takes a subcode 0 to %u\r\n", m, (unsigned)N - 1);
            return;
        }

        const ParamRow& row = rows[sub];
        // one byte more than the longest value, a 64-byte password, so it always ends in a NUL
        u8 value[65]{};
        u8 len = 0;
        u16 status = 0;
        u8 ok = locked([&] {
            if (row.kind == HOSTNAME)
                return M8266WIFI_SPI_Get_STA_Hostname((char*)value, &status);

            if (sta)
                return M8266WIFI_SPI_Query_STA_Param((STA_PARAM_TYPE)row.type, value, &len,
                                                     &status);

            return M8266WIFI_SPI_Query_AP_Param((AP_PARAM_TYPE)row.type, value, &len, &status);
        });
        if (!ok) {
            module_error(row.name, status);
            return;
        }

        if (row.kind == MAC) {
            printk("%s: %02X:%02X:%02X:%02X:%02X:%02X\r\n", row.name, value[0], value[1],
                   value[2], value[3], value[4], value[5]);
        } else if (row.kind == NUMBER) {
            printk("%s: %u\r\n", row.name, value[0]);
        } else {
            printk("%s: %s\r\n", row.name, (const char*)value);
        }
    }
}

void WifiProvider::query_sta_param(Gcode *gcode)
{
    query_param(482, sta_params, true, gcode->subcode());
}

void WifiProvider::query_ap_param(Gcode *gcode)
{
    query_param(483, ap_params, false, gcode->subcode());
}

void WifiProvider::report_status(Gcode *gcode)
{
    query_wifi_status();
}

void WifiProvider::int_to_ip(uint32_t i_ip, char* ip_addr)
{
    unsigned int bytes[4];
    bytes[0] = (i_ip >> 24) & 0xFF;
    bytes[1] = (i_ip >> 16) & 0xFF;
    bytes[2] = (i_ip >> 8) & 0xFF;
    bytes[3] = i_ip & 0xFF;
    snprintf(ip_addr, 16, "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3]);
}

uint32_t WifiProvider::ip_to_int(char* ip_addr)
{
    uint32_t ip = 0;
    char *p = ip_addr;
    for (int i = 0; i < 4; i++) {
        ip = (ip << 8) | (strtoul(p, &p, 10) & 0xFF);
        if (*p == '.') p++;
    }
    return ip;
}

void WifiProvider::on_second_tick(void*)
{
    u16 status = 0;
    char address[16];
    char udp_buff[100];
    u8 param_len = 0;
    u8 connection_status = 0;
    u8 client_num = 0;
    ClientInfo RemoteClients[15];

    if (!wifi_init_ok) return;

    bool listed = false;
    {
        ModuleLock lock;
        listed = M8266WIFI_SPI_List_Clients_On_A_TCP_Server(tcp_link_no, &client_num, RemoteClients, &status);
        M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
    }
    if (listed) reap_sessions(RemoteClients, client_num);

    if (connection_status == 5) {
        // Connected to AP
        // Get IP and netmask
        {
            ModuleLock lock;
            M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_IP_ADDR, (u8*)this->sta_address, &param_len, &status);
            M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_NETMASK_ADDR, (u8*)this->sta_netmask, &param_len, &status);

            get_broadcast_from_ip_and_netmask(address, this->sta_address, this->sta_netmask);
            snprintf(udp_buff, sizeof(udp_buff), "%s,%s,%d,%d", this->machine_name, this->sta_address, this->tcp_port, client_num > 0 ? 1 : 0);
            M8266WIFI_SPI_Send_Udp_Data((u8*)udp_buff, strlen(udp_buff), udp_link_no, address, this->udp_send_port, &status);
        }
        connection_fail_count = 0;
    } else if (connection_status == 2 || connection_status == 3 || connection_status == 4) {
        // Connection failed
        connection_fail_count++;
        if (connection_fail_count > 10) {
            // Disconnect WiFi
            if (M8266WIFI_SPI_STA_DisConnect_Ap(&status)) {
                printk("STA connection timeout, disconnected!\n");
            }
            connection_fail_count = 0;
        }
    } else {
        connection_fail_count = 0;
    }

    // Send AP info through UDP
    memset(udp_buff, 0, sizeof(udp_buff));
    get_broadcast_from_ip_and_netmask(address, this->ap_address, this->ap_netmask);
    snprintf(udp_buff, sizeof(udp_buff), "%s,%s,%d,%d", this->machine_name, this->ap_address, this->tcp_port, client_num > 0 ? 1 : 0);
    {
        ModuleLock lock;
        M8266WIFI_SPI_Send_Udp_Data((u8*)udp_buff, strlen(udp_buff), udp_link_no, address, this->udp_send_port, &status);
    }
}

void WifiProvider::service()
{
    if (beacon_due) {
        beacon_due= false;
        on_second_tick(nullptr);
    }

    bool pending;
    {
        ModuleLock lock;
        pending = M8266WIFI_SPI_Has_DataReceived();
    }

    if (has_data_flag || pending) {
        has_data_flag = false;
        while (read_chunk(true)) { }
    }

    // every client gets its turn: one client's lines must not wait on another's traffic
    for (Session& s : sessions) {
        if (s.live() && s.accept_event()) s.pump();
    }
}

// Broadcast: the kernel's pool hands this the frames to put on every console. Sessions are
// not pool members themselves -- the set is walked from other tasks without a lock.
int WifiProvider::puts(const char* s, int size)
{
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    flush_unlocked(false);   // pending replies first, so the line cannot overtake them; this
                             // runs off the main loop and must not sit on a stalled client
    xSemaphoreGive(tx_lock);
    int r = 0;
    size_t n = size == 0 ? strlen(s) : size;
    for (Session& sess : sessions) {
        // a stalled session dropped this line: keep it out until it reads again, or every
        // broadcast pays the module's blocked send
        if (!sess.live() || sess.stall != 0 || !sess.accept_event()) continue;
        size_t got = send_to_client(sess.who.ip, sess.who.port, sess.link, (const u8*)s, n,
                                    WIFI_TX_RETRIES_SHORT);
        if (got != n) sess.stall = Session::STALL_TICKS;
        if ((int)got > r) r = (int)got;
    }
    return r;
}

u16 WifiProvider::send_to_client(const u8 ip[4], u16 port, u8 link, const u8* data, size_t len,
                                 int retries)
{
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

    size_t sent_index = 0;
    u16 status = 0;
    while (sent_index < len) {
        u16 to_send = std::min<size_t>(len - sent_index, WIFI_DATA_MAX_SIZE);
        u16 sent;
        {
            ModuleLock lock;
            sent = M8266WIFI_SPI_Send_Data_to_TcpClient((u8*)(data + sent_index), to_send, link,
                                                        ip_str, port, &status);
        }
        sent_index += sent;
        if (sent == to_send) continue;

        // 0x11 (waiting for wifi to send) and 0x12 (send buffer full) mean busy: the module
        // drains by itself, so resend the remainder. Anything else is a dead client, give up.
        u8 err = status & 0xFF;
        if ((err == 0x11 || err == 0x12) && retries-- > 0) {
            taskYIELD();   // let the module drain and other tasks run, but try again now
            continue;
        }
        break;
    }
    return sent_index;
}


void WifiProvider::set_wifi_op_mode(u8 op_mode)
{
    u16 status = 0;
    u8 ok = locked([&] { return M8266WIFI_SPI_Set_Opmode(op_mode, 1, &status); });
    if (ok == 0) {
        module_error("set opmode", status);
    } else if (op_mode == 1) {
        printk("WiFi Access Point Disabled...\n");
    } else if (op_mode == 3) {
        printk("WiFi Access Point Enabled...\n");
    }
}

std::string WifiProvider::scan_wlans()
{

    u8 signals = 0;
    u16 status = 0;
    char ssid[32];
    u8 ssid_len = 0;
    u8 connection_status = 0;

    // Get current connected SSID
    M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_SSID, (u8*)ssid, &ssid_len, &status);
    M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);

    ScannedSigs wlans[MAX_WLAN_SIGNALS];
    // Start scanning for WLAN signals
    M8266WIFI_SPI_STA_Scan_Signals(wlans, MAX_WLAN_SIGNALS, 0xff, 0, &status);
    // Wait for scan to finish
    while (true) {
        signals = M8266WIFI_SPI_STA_Fetch_Last_Scanned_Signals(wlans, MAX_WLAN_SIGNALS, &status);
        if (signals == 0) {
            if ((status & 0xff) == 0x26) {
                // Still scanning; wait
                delay_ms(1);
                continue;
            } else {
                // Scan failed
                return std::string();
            }
        } else {
            // Prepare data for public request
            size_t n;
            std::string str;
            std::string ssid_str;
            char buf[10];
            for (int i = 0; i < signals; i++) {
                ssid_str = "";
                for (size_t j = 0; j < strlen(wlans[i].ssid); j++) {
                    ssid_str += wlans[i].ssid[j] == ' ' ? 0x01 : wlans[i].ssid[j];
                }
                ssid_str.append(",");
                // Ignore duplicate SSIDs
                if (str.find(ssid_str) != std::string::npos) {
                    continue;
                }
                str.append(ssid_str);
                str.append(wlans[i].authmode == 0 ? "0" : "1");
                str.append(",");
                n = snprintf(buf, sizeof(buf), "%d", wlans[i].rssi);
                if (n > sizeof(buf)) n = sizeof(buf);
                str.append(buf, n);
                str.append(",");
                if (strncmp(ssid, wlans[i].ssid, ssid_len <= 32 ? ssid_len : 32) == 0 && connection_status == 5) {
                    str.append("1\n");
                } else {
                    str.append("0\n");
                }
            }
            return str;
        }
    }
    return std::string();
}

void WifiProvider::connect_ap(struct ap_conn_info *s)
{
    u16 status = 0;
    u8 connection_status;

    s->has_error = false;
    if (s->disconnect) {
        // Disconnect from AP
        if (M8266WIFI_SPI_STA_DisConnect_Ap(&status) == 0) {
            s->has_error = true;
            snprintf(s->error_info, sizeof(s->error_info), "Disconnect error!");
        }
    } else {
        // Connect to AP
        M8266WIFI_SPI_STA_Connect_Ap((u8*)s->ssid, (u8*)s->password, 1, 0, &status);
        // Wait for connection
        while (true) {
            M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
            if (connection_status == 1) {
                // Connecting; wait
                delay_ms(1);
                continue;
            } else if (connection_status == 5) {
                // Connection successful
                s->has_error = false;
                break;
            } else {
                // Connection failed
                s->has_error = true;
                if (connection_status == 0) {
                    snprintf(s->error_info, sizeof(s->error_info), "No connection started!");
                } else if (connection_status == 2) {
                    snprintf(s->error_info, sizeof(s->error_info), "WiFi password incorrect!");
                } else if (connection_status == 3) {
                    snprintf(s->error_info, sizeof(s->error_info), "WiFi SSID not found: %s!", s->ssid);
                } else if (connection_status == 4) {
                    snprintf(s->error_info, sizeof(s->error_info), "Other error!");
                }
                break;
            }
        }
        // Get IP address if connected
        if (!s->has_error) {
            M8266WIFI_SPI_Get_STA_IP_Addr(s->ip_address, &status);
        }
    }
}

bool WifiProvider::config_ap(AP_PARAM_TYPE type, u8* value, u8 len, const char* what)
{
    u16 status = 0;
    u8 ok = locked([&] { return M8266WIFI_SPI_Config_AP_Param(type, value, len, 1, &status); });
    if (!ok)
        module_error(what, status);

    return ok;
}

void WifiProvider::set_ap_channel(uint8_t channel)
{
    if (config_ap(AP_PARAM_TYPE_CHANNEL, &channel, 1, "set AP channel"))
        printk("WiFi AP Channel changed to %d\n", channel);
}

void WifiProvider::set_ap_ssid(const char *ssid)
{
    if (config_ap(AP_PARAM_TYPE_SSID, (u8*)ssid, strlen(ssid), "set AP SSID"))
        printk("WiFi AP SSID changed to %s\n", ssid);
}

void WifiProvider::set_ap_password(const char *password)
{
    u16 status = 0;
    u8 op_mode;
    if (!locked([&] { return M8266WIFI_SPI_Get_Opmode(&op_mode, &status); })) {
        module_error("opmode query", status);
        return;
    }

    if (op_mode != 3) {
        printk("WiFi cannot set password when not in AP mode!\n");
        return;
    }

    u8 authmode = strlen(password) == 0 ? 0 : 4;
    if (config_ap(AP_PARAM_TYPE_PASSWORD, (u8*)password, strlen(password), "set AP password"))
        printk("WiFi AP Password changed to %s\n", password);

    config_ap(AP_PARAM_TYPE_AUTHMODE, &authmode, 1, "set AP authmode");
}

void WifiProvider::set_ap_enabled(bool on)
{
    if (on) {
        set_wifi_op_mode(3);
    } else {
        set_wifi_op_mode(1);
    }
}

void WifiProvider::query_wifi_status()
{
    u16 status = 0;
    u32 esp8266_id;
    u8 flash_size;
    char fw_ver[24] = "";
    printk("M8266WIFI_SPI_Get_Module_Info...\n");
    u8 ok = locked([&] {
        return M8266WIFI_SPI_Get_Module_Info(&esp8266_id, &flash_size, fw_ver, &status);
    });
    if (ok == 0) {
        module_error("module info", status);
    } else {
        printk("esp8266_id:%ld, flash_size:%d, fw_ver:%s!\n", esp8266_id, flash_size, fw_ver);
    }
}

void WifiProvider::init_wifi_module(bool reset)
{
    u16 status = 0;
    char address[16];
    u8 param_len = 0;

    if (reset) {
        // Reset module: delete connections and remove stream
        M8266WIFI_SPI_Delete_Connection(udp_link_no, &status);
        M8266WIFI_SPI_Delete_Connection(tcp_link_no, &status);
        THEKERNEL->streams.remove_stream(this);
    }

    // Initialize module via SPI
    M8266HostIf_Init();
    if (M8266WIFI_Module_Init_Via_SPI() == 0) {
        printk("error:wifi module init failed\n");
    }

    // Set up TCP and UDP connections
    snprintf(address, sizeof(address), "192.168.4.10");
    if (M8266WIFI_SPI_Setup_Connection(2, this->tcp_port, address, 0, tcp_link_no, 3, &status) == 0) {
        module_error("console server setup", status);
    }
    snprintf(address, sizeof(address), "192.168.4.255");
    if (M8266WIFI_SPI_Setup_Connection(0, this->udp_recv_port, address, 0, udp_link_no, 3, &status) == 0) {
        module_error("beacon link setup", status);
    }

    if (M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(tcp_link_no, MAX_SESSIONS, &status) == 0) {
        module_error("console server max clients", status);
    }

    // Set TCP server auto-disconnect timeout
    if (M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(tcp_link_no, this->tcp_timeout_s, &status) == 0) {
        module_error("console server timeout", status);
    }

    // Load current AP IP and Netmask
    if (M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_IP_ADDR, (u8*)this->ap_address, &param_len, &status) == 0) {
        module_error("AP address query", status);
    }
    if (M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_NETMASK_ADDR, (u8*)this->ap_netmask, &param_len, &status) == 0) {
        module_error("AP netmask query", status);
    }

    if (reset) {
        // Re-append stream after reset
        THEKERNEL->streams.append_stream(this);
    }

    wifi_init_ok = true;
}

void WifiProvider::M8266WIFI_Module_Hardware_Reset(void) // total 800ms  (Chinese: 本例子中这个函数的总共执行时间大约800毫秒)
{
	M8266HostIf_Set_SPI_nCS_Pin(0);   			// Module nCS==ESP8266 GPIO15 as well, Low during reset in order for a normal reset (Chinese: 为了实现正常复位，模块的片选信号nCS在复位期间需要保持拉低)
	delay_ms(1); 	    		// delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

	M8266HostIf_Set_nRESET_Pin(0);					// Pull low the nReset Pin to bring the module into reset state (Chinese: 拉低nReset管脚让模组进入复位状态)
	delay_ms(5);      		// delay 5ms, adequate for nRESET stable(Chinese: 延迟5毫秒，确保片选nRESER设置后有足够的时间来稳定，也确保nCS和nRESET有足够的时间同时处于低电平状态)
	                                        // give more time especially for some board not good enough
	                                        //(Chinese: 如果主板不是很好，导致上升下降过渡时间较长，或者因为失配存在较长的振荡时间，所以信号到轨稳定的时间较长，那么在这里可以多给一些延时)

	M8266HostIf_Set_nRESET_Pin(1);					// Pull high again the nReset Pin to bring the module exiting reset state (Chinese: 拉高nReset管脚让模组退出复位状态)
	delay_ms(300); 	  		// at least 18ms required for reset-out-boot sampling boottrap pin (Chinese: 至少需要18ms的延时来确保退出复位时足够的boottrap管脚采样时间)
	                                        // Here, we use 300ms for adequate abundance, since some board GPIO, (Chinese: 在这里我们使用了300ms的延时来确保足够的富裕量，这是因为在某些主板上，)
																					// needs more time for stable(especially for nRESET) (Chinese: 他们的GPIO可能需要较多的时间来输出稳定，特别是对于nRESET所对应的GPIO输出)
																					// You may shorten the time or give more time here according your board v.s. effiency
																					// (Chinese: 如果你的主机板在这里足够好，你可以缩短这里的延时来缩短复位周期；反之则需要加长这里的延时。
																					//           总之，你可以调整这里的时间在你们的主机板上充分测试，找到一个合适的延时，确保每次复位都能成功。并适当保持一些富裕量，来兼容批量化时主板的个体性差异)
	M8266HostIf_Set_SPI_nCS_Pin(1);         // release/pull-high(defualt) nCS upon reset completed (Chinese: 释放/拉高(缺省)片选信号
	//delay_ms(1); 	    		// delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

	delay_ms(800-300-5-2); // Delay more than around 500ms for M8266WIFI module bootup and initialization，including bootup information print。No influence to host interface communication. Could be shorten upon necessary. But test for verification required if adjusted.
	                                        // (Chinese: 延迟大约500毫秒，来等待模组成功复位后完成自己的启动过程和自身初始化，包括串口信息打印。但是此时不影响模组和单片主机之间的通信，这里的时间可以根据需要适当调整.如果调整缩短了这里的时间，建议充分测试，以确保系统(时序关系上的)可靠性)
}

u8 WifiProvider::M8266WIFI_Module_Init_Via_SPI()
{
    u16 status = 0;
    uint32_t spi_clk = 24000000;

    // Step 1: Hardware reset the module
    M8266WIFI_Module_Hardware_Reset();

    // Step 2: Set SPI clock speed
	#ifndef SPI_BaudRatePrescaler_2
	#define SPI_BaudRatePrescaler_2         ((u32)0x00000002U)
	#define SPI_BaudRatePrescaler_4         ((u32)0x00000004U)
	#define SPI_BaudRatePrescaler_6         ((u32)0x00000006U)
	#define SPI_BaudRatePrescaler_8         ((u32)0x00000008U)
	#define SPI_BaudRatePrescaler_16        ((u32)0x00000010U)
	#define SPI_BaudRatePrescaler_32        ((u32)0x00000020U)
	#define SPI_BaudRatePrescaler_64        ((u32)0x00000040U)
	#define SPI_BaudRatePrescaler_128       ((u32)0x00000080U)
	#define SPI_BaudRatePrescaler_256       ((u32)0x00000100U)
	#endif	
    M8266HostIf_SPI_SetSpeed(SPI_BaudRatePrescaler_4);
    spi_clk = 24000000;
    delay_ms(1);

    // Step 3: Select SPI interface
    if (M8266HostIf_SPI_Select((uint32_t)M8266WIFI_INTERFACE_SPI, spi_clk, &status) == 0) {
        module_error("SPI select", status);
        return 0;
    }

    // Step 4: Communication test
    u8 byte;
    if (M8266WIFI_SPI_Interface_Communication_OK(&byte) == 0) {
        printk("error:wifi SPI test failed\n");
        return 0;
    }

    const int tries = 100000;
    int passed = M8266WIFI_SPI_Interface_Communication_Stress_Test(tries);
    if (passed < tries && tries - passed > 5) {
        printk("error:wifi SPI stress test failed\n");
        return 0;
    }

    // Step 5: Configure module
    if (M8266WIFI_SPI_Set_Tx_Max_Power(68, &status) == 0) {
        module_error("set TX power", status);
        return 0;
    }

    return 1;
}

/* API */

uint8_t WifiProvider::getNextLinkNo() {
    return next_available_link_no++;
}

uint8_t WifiProvider::initializeTcpServer(uint16_t local_port, uint8_t max_clients)
{
    uint16_t status = 0;
    const int connection_type = 2; // TCP Server
    const int timeout = 3;
    uint8_t link_no = getNextLinkNo();

    // Setup the connection
    if (M8266WIFI_SPI_Setup_Connection(connection_type, local_port, const_cast<char*>("0.0.0.0"), 0, link_no, timeout, &status) == 0) {
        module_error("TCP server setup", status);
        return 0xFF;
    }

    // Configure the maximum number of clients allowed for a TCP server
    if (connection_type == 2) {
        if (M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(link_no, max_clients, &status) == 0) {
            module_error("TCP server max clients", status);
            return 0xFF;
        }
    }

    return link_no;
}

void WifiProvider::registerTcpDataCallback(uint8_t link_no, std::function<void(uint8_t *, uint16_t, uint8_t*, uint16_t)> callback)
{
    data_callbacks[link_no] = callback;
}

bool WifiProvider::sendTcpDataToClient(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no, const uint8_t* data, uint16_t length)
{
    return send_to_client(remote_ip, remote_port, link_no, data, length) == length;
}

bool WifiProvider::closeTcpConnection(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no)
{
    uint16_t status = 0;
    uint8_t client_num = 0;
    ClientInfo RemoteClients[15]; // Adjust the size based on maximum expected clients

    // Get the list of clients connected to the TCP server
    if (M8266WIFI_SPI_List_Clients_On_A_TCP_Server(link_no, &client_num, RemoteClients, &status) == 0) {
        module_error("client list", status);
        return false;
    }

    // Iterate through the clients to find the matching one
    for (uint8_t i = 0; i < client_num; ++i) {
        ClientInfo& client = RemoteClients[i];

        // Compare IP addresses and port
        if (memcmp(client.remote_ip, remote_ip, 4) == 0 && client.remote_port == remote_port) {
            // Found the matching client, disconnect it
            if (M8266WIFI_SPI_Disconnect_TcpClient(link_no, &client, &status) == 0) {
                module_error("client disconnect", status);
                return false;
            }
            return true; // Disconnected successfully
        }
    }

    // Client not found
    printk("Client not found on link %d\n", link_no);
    return false;
}
