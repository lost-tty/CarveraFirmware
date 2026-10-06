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
#include "libs/MainWake.h"
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
#include "port_api.h"
#include "InterruptIn.h"

#include "gpio.h"
#include "libs/DeferredWake.h"

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
static void module_error(const char* what, u16 status, StreamOutput* to = nullptr)
{
    if (to)
        to->printf("error:wifi %s failed, status 0x%04x\r\n", what, status);
    else
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
    connection_fail_count = 0;

    const WifiConfig &wifi_config = ConfigTable::config<WifiConfig>(wifi_provider_config_groups);
    configure(&wifi_config);
    if (!wifi_config.enable) {
        // Not needed; free up resources
        return;
    }

	data_callbacks.clear();


    // Initialize WiFi module
    this->init_wifi_module();

    // high while the module holds data; without it the link asks the module every few ms
    Pin pin;
    pin.from_spec(wifi_config.interrupt_pin);
    pin.as_input();
    mbed::InterruptIn* data_pin = nullptr;
    if (pin.port_number == 0 || pin.port_number == 2) {
        data_pin = new mbed::InterruptIn(port_pin((PortName)pin.port_number, pin.pin));
        NVIC_SetPriority(EINT3_IRQn, 16);
    } else {
        printk("Error: WiFi interrupt pin must be on P0 or P2.\n");
    }
    console.attach(&link, tcp_link_no);
    link.start(tcp_link_no, data_pin);

    SimpleShell::add_command(shell_slot, "wifi", &WifiProvider::shell, this,
                             "wifi webserver|tcp|clients - the module's web page, TCP, clients");

    // Register for events
    ADD_MCODE(m482, 482, IMMEDIATE, WifiProvider::query_sta_param);
    ADD_MCODE(m483, 483, IMMEDIATE, WifiProvider::query_ap_param);
    ADD_MCODE(m489, 489, IMMEDIATE, WifiProvider::report_status);
    beacon.start();
}

void WifiProvider::get_broadcast_from_ip_and_netmask(char* broadcast_addr, char* ip_addr,
                                                     char* netmask)
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

const SimpleShell::Sub<WifiProvider> WifiProvider::SUBS[] = {
    {"webserver", &WifiProvider::sub_webserver, "on [port] | off - the module's own web server"},
    {"clients", &WifiProvider::sub_clients, "- the console sessions and the module's client list"},
    {"tcp", &WifiProvider::sub_tcp, "- the console server's TCP window"},
    {nullptr, nullptr, nullptr},
};

void WifiProvider::shell(void* self, const char* cmd, std::string args, StreamOutput* stream)
{
    SimpleShell::dispatch(static_cast<WifiProvider*>(self), SUBS, cmd, args, stream);
}

// the window can only be set at boot: a connection keeps the window its server had when it
// connected
void WifiProvider::sub_tcp(std::string, StreamOutput* stream)
{
    u16 status = 0;
    u8 window = 0;
    u8 ok = locked([&] {
        return M8266WIFI_SPI_Query_Tcp_Window_num(tcp_link_no, &window, &status);
    });
    if (!ok) {
        module_error("TCP window query", status, stream);
        return;
    }
    stream->printf("wifi tcp window %u segments, %u asked for at boot\r\n", window,
                   WIFI_TCP_WINDOW);
}

void WifiProvider::sub_clients(std::string, StreamOutput* stream)
{
    console.list(stream);
    stream->printf("rx queued: %s, module lists %u clients, last list status 0x%04x\n",
                   link.rx_head() ? "yes" : "no", list_count, list_status);
}

// the module's built-in web page for the WLAN setup, not the firmware's web server; "on"
// without a port uses the port saved on the module, and nothing is saved there
void WifiProvider::sub_webserver(std::string args, StreamOutput* stream)
{
    std::string what = shift_parameter(args);
    bool on = what == "on";
    if (!on && what != "off") {
        stream->printf("usage: wifi webserver on [port] | off\r\n");
        return;
    }

    unsigned long want = 0;
    if (on && !args.empty()) {
        char* end;
        want = strtoul(args.c_str(), &end, 10);
        if (*end != '\0' || want < 1 || want > 65535) {
            stream->printf("error:port must be 1 to 65535\r\n");
            return;
        }
    }

    u8 boot = 0, running = 0;
    u16 saved_port = 0, port = 0, status = 0;
    u8 ok = locked([&] {
        return M8266WIFI_SPI_Query_WebServer(&boot, &running, &saved_port, &port, &status);
    });
    if (!ok) {
        module_error("web server query", status, stream);
        return;
    }

    if (want == 0)
        want = saved_port;
    ok = locked([&] {
        return M8266WIFI_SPI_Set_WebServer(on, on ? want : port, 0, &status)
               && M8266WIFI_SPI_Query_WebServer(&boot, &running, &saved_port, &port, &status);
    });
    if (!ok) {
        module_error("web server", status, stream);
        return;
    }

    if (running)
        stream->printf("wifi webserver on, port %u\r\n", port);
    else
        stream->printf("wifi webserver off\r\n");
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
        if (*p == '.')
            p++;
    }
    return ip;
}

void WifiProvider::on_beacon()
{
    beacon_due = true;
    wake_main();
}

// Once a second, on the main loop: the client list for reaping, the station's state, and the
// beacon that lets clients find the machine.
void WifiProvider::on_second_tick()
{
    u16 status = 0;
    u8 param_len = 0;
    u8 connection_status = 0;
    u8 client_num = 0;
    ClientInfo clients[15];

    if (!wifi_init_ok)
        return;

    bool listed = false;
    {
        ModuleLock lock;
        listed = M8266WIFI_SPI_List_Clients_On_A_TCP_Server(tcp_link_no, &client_num, clients,
                                                            &status);
        list_status = listed ? 0 : status;
        list_count = client_num;
        M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
    }
    if (listed)
        console.reap(clients, std::min<u8>(client_num, 15));

    if (connection_status == 5) {
        // joined: the beacon goes out on the station's network too
        {
            ModuleLock lock;
            M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_IP_ADDR, (u8*)sta_address, &param_len,
                                          &status);
            M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_NETMASK_ADDR, (u8*)sta_netmask,
                                          &param_len, &status);
        }
        send_beacon(sta_address, sta_netmask, client_num);
        connection_fail_count = 0;
    } else if (connection_status == 2 || connection_status == 3 || connection_status == 4) {
        // Connection failed
        connection_fail_count++;
        if (connection_fail_count > 10) {
            // Disconnect WiFi
            u8 dropped;
            dropped = locked([&] { return M8266WIFI_SPI_STA_DisConnect_Ap(&status); });
            if (dropped) {
                printk("STA connection timeout, disconnected!\n");
            }
            connection_fail_count = 0;
        }
    } else {
        connection_fail_count = 0;
    }

    send_beacon(ap_address, ap_netmask, client_num);
}

// "name,ip,port,busy" to the network's broadcast address: how clients find the machine.
void WifiProvider::send_beacon(char* ip, char* netmask, u8 clients)
{
    char address[16];
    char beacon[100];
    get_broadcast_from_ip_and_netmask(address, ip, netmask);
    snprintf(beacon, sizeof(beacon), "%s,%s,%d,%d", machine_name, ip, tcp_port,
             clients > 0 ? 1 : 0);
    u16 status = 0;
    ModuleLock lock;
    M8266WIFI_SPI_Send_Udp_Data((u8*)beacon, strlen(beacon), udp_link_no, address,
                                udp_send_port, &status);
}

// The main loop's side: the second's housekeeping, what arrived (the console link's to the
// console server, other links' to their callbacks), then each session's commands.
void WifiProvider::service()
{
    // wifi disabled in the config: no task, no queues
    if (!link.running())
        return;

    if (beacon_due) {
        beacon_due = false;
        on_second_tick();
    }

    while (WifiRxBuf* b = link.rx_head()) {
        if (b->off < b->len) {
            if (b->link == tcp_link_no) {
                if (!console.take(*b))
                    break;
            } else {
                auto it = data_callbacks.find(b->link);
                if (it != data_callbacks.end())
                    it->second(b->ip, b->port, b->data, b->len);
            }
        }
        link.rx_release();
    }
    console.pump();
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
    {
        ModuleLock lock;
        M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_SSID, (u8*)ssid, &ssid_len, &status);
        M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
    }

    ScannedSigs wlans[MAX_WLAN_SIGNALS];
    // Start scanning for WLAN signals
    locked([&] {
        return M8266WIFI_SPI_STA_Scan_Signals(wlans, MAX_WLAN_SIGNALS, 0xff, 0, &status);
    });
    // Wait for scan to finish
    while (true) {
        signals = locked([&] {
            return M8266WIFI_SPI_STA_Fetch_Last_Scanned_Signals(wlans, MAX_WLAN_SIGNALS, &status);
        });
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
                if (n > sizeof(buf))
                    n = sizeof(buf);

                str.append(buf, n);
                str.append(",");
                if (strncmp(ssid, wlans[i].ssid, ssid_len <= 32 ? ssid_len : 32) == 0
                    && connection_status == 5) {
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
        u8 ok;
        ok = locked([&] { return M8266WIFI_SPI_STA_DisConnect_Ap(&status); });
        if (ok == 0) {
            s->has_error = true;
            snprintf(s->error_info, sizeof(s->error_info), "Disconnect error!");
        }
    } else {
        // Connect to AP
        locked([&] {
            return M8266WIFI_SPI_STA_Connect_Ap((u8*)s->ssid, (u8*)s->password, 1, 0, &status);
        });
        // Wait for connection
        while (true) {
            locked([&] {
                return M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
            });
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
                    snprintf(s->error_info, sizeof(s->error_info), "WiFi SSID not found: %s!",
                             s->ssid);
                } else if (connection_status == 4) {
                    snprintf(s->error_info, sizeof(s->error_info), "Other error!");
                }
                break;
            }
        }
        // Get IP address if connected
        if (!s->has_error) {
            ModuleLock lock;
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

void WifiProvider::init_wifi_module()
{
    u16 status = 0;
    u8 param_len = 0;

    M8266HostIf_Init();
    if (M8266WIFI_Module_Init_Via_SPI() == 0) {
        printk("error:wifi module init failed\n");
    }

    // the router gets this name by DHCP, from the module's next lease on
    if (M8266WIFI_SPI_Set_STA_Hostname(this->machine_name, &status) == 0) {
        module_error("set hostname", status);
    }

    // a TCP server ignores the remote address
    if (M8266WIFI_SPI_Setup_Connection(2, tcp_port, const_cast<char*>("0.0.0.0"), 0, tcp_link_no,
                                       3, &status) == 0) {
        module_error("console server setup", status);
    }
    // The module's default window of 2 segments makes an upload wait for every ACK. A
    // connection keeps the window its server had when it connected, so this comes first.
    if (M8266WIFI_SPI_Config_Tcp_Window_num(tcp_link_no, WIFI_TCP_WINDOW, &status) == 0) {
        module_error("set TCP window", status);
    }
    if (M8266WIFI_SPI_Setup_Connection(0, udp_recv_port, const_cast<char*>("192.168.4.255"), 0,
                                       udp_link_no, 3, &status) == 0) {
        module_error("beacon link setup", status);
    }

    if (M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(tcp_link_no, MAX_SESSIONS,
                                                                  &status) == 0) {
        module_error("console server max clients", status);
    }
    if (M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(tcp_link_no, tcp_timeout_s,
                                                        &status) == 0) {
        module_error("console server timeout", status);
    }

    // the beacon's broadcast address comes from these
    if (M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_IP_ADDR, (u8*)ap_address, &param_len,
                                     &status) == 0) {
        module_error("AP address query", status);
    }
    if (M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_NETMASK_ADDR, (u8*)ap_netmask, &param_len,
                                     &status) == 0) {
        module_error("AP netmask query", status);
    }

    wifi_init_ok = true;
}

// The vendor's reset, about 800 ms: nCS (also the ESP8266's GPIO15 boot strap) is held low
// through it, the module samples its boot straps for at least 18 ms after nRESET rises, and
// needs about 500 ms more to boot. The margins are the vendor's, for slow board GPIOs.
void WifiProvider::M8266WIFI_Module_Hardware_Reset(void)
{
    M8266HostIf_Set_SPI_nCS_Pin(0);
    delay_ms(1);
    M8266HostIf_Set_nRESET_Pin(0);
    delay_ms(5);
    M8266HostIf_Set_nRESET_Pin(1);
    delay_ms(300);
    M8266HostIf_Set_SPI_nCS_Pin(1);
    delay_ms(800 - 300 - 5 - 2);
}

u8 WifiProvider::M8266WIFI_Module_Init_Via_SPI()
{
    u16 status = 0;

    // CPU clock / 2, the SSP's fastest (about 48 MHz); a board or module that fails the
    // stress test there gets / 4, about 24 MHz, as before
    // CPU / 4, 25 MHz: the Carvera's module fails at / 2
    const u32 prescaler = 4;
    M8266WIFI_Module_Hardware_Reset();
    M8266HostIf_SPI_SetSpeed(prescaler);
    delay_ms(1);
    if (M8266HostIf_SPI_Select((uint32_t)M8266WIFI_INTERFACE_SPI, SystemCoreClock / prescaler,
                               &status) == 0) {
        module_error("SPI select", status);
        return 0;
    }

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
    u8 ok;

    // Setup the connection
    ok = locked([&] {
        return M8266WIFI_SPI_Setup_Connection(connection_type, local_port,
                                              const_cast<char*>("0.0.0.0"), 0, link_no, timeout,
                                              &status);
    });
    if (ok == 0) {
        module_error("TCP server setup", status);
        return 0xFF;
    }

    // Configure the maximum number of clients allowed for a TCP server
    if (connection_type == 2) {
        ok = locked([&] {
            return M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(link_no, max_clients,
                                                                            &status);
        });
        if (ok == 0) {
            module_error("TCP server max clients", status);
            return 0xFF;
        }
    }

    return link_no;
}

void WifiProvider::removeTcpServer(uint8_t link_no)
{
    u16 status = 0;
    data_callbacks.erase(link_no);
    ModuleLock lock;
    M8266WIFI_SPI_Delete_Connection(link_no, &status);
}

void WifiProvider::registerTcpDataCallback(uint8_t link_no,
    std::function<void(uint8_t *, uint16_t, uint8_t*, uint16_t)> callback)
{
    data_callbacks[link_no] = callback;
}

bool WifiProvider::sendTcpDataToClient(const uint8_t* remote_ip, uint16_t remote_port,
                                       uint8_t link_no, const uint8_t* data, uint16_t length)
{
    return link.send_to_client(remote_ip, remote_port, link_no, data, length) == length;
}

bool WifiProvider::closeTcpConnection(const uint8_t* remote_ip, uint16_t remote_port,
                                      uint8_t link_no)
{
    uint16_t status = 0;
    uint8_t client_num = 0;
    ClientInfo RemoteClients[15]; // Adjust the size based on maximum expected clients
    u8 ok;

    // Get the list of clients connected to the TCP server
    ok = locked([&] {
        return M8266WIFI_SPI_List_Clients_On_A_TCP_Server(link_no, &client_num, RemoteClients,
                                                          &status);
    });
    if (ok == 0) {
        module_error("client list", status);
        return false;
    }

    // Iterate through the clients to find the matching one
    for (uint8_t i = 0; i < client_num; ++i) {
        ClientInfo& client = RemoteClients[i];

        // Compare IP addresses and port
        if (memcmp(client.remote_ip, remote_ip, 4) == 0 && client.remote_port == remote_port) {
            // Found the matching client, disconnect it
            ok = locked([&] {
                return M8266WIFI_SPI_Disconnect_TcpClient(link_no, &client, &status);
            });
            if (ok == 0) {
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
