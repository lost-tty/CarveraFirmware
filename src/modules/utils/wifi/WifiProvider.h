/*
 * WifiProvider.h
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#ifndef WIFIPROVIDER_H_
#define WIFIPROVIDER_H_

#include <map>
#include <functional>
#include <string>

#include "Module.h"
#include "SoftTimer.h"
#include "StreamOutput.h"
#include "libs/McodeRegistry.h"
#include "SimpleShell.h"

#include "M8266WIFIDrv.h"
#include "WifiLink.h"
#include "modules/communication/ConsoleServer.h"

#define MAX_WLAN_SIGNALS 8
#define WIFI_TCP_WINDOW 8                // the console server's receive window, in segments

class Gcode;

// A network to join, and how joining it went (the wlan shell command).
struct ap_conn_info {
    char ssid[32];
    char password[64];
    char ip_address[16];
    bool has_error;
    char error_info[64];
    bool disconnect;
};

// The M8266 module: its setup, the networks it joins or offers, the console server on its
// TCP link and the TCP servers other modules ask for. The wifi task (WifiLink) moves the bytes;
// the console server (ConsoleServer) holds the console clients.
class WifiProvider : public Module
{
public:
    std::string scan_wlans();
    void connect_ap(struct ap_conn_info *s);
    void set_ap_channel(uint8_t channel);
    void set_ap_ssid(const char *ssid);
    void set_ap_password(const char *password);
    void set_ap_enabled(bool on);
    void on_module_loaded();
    void service();
    void configure(const void *cfg);

    uint8_t initializeTcpServer(uint16_t local_port, uint8_t max_clients);
    void removeTcpServer(uint8_t link_no);
    void registerTcpDataCallback(uint8_t link_no, std::function<void(uint8_t*, uint16_t, uint8_t*, uint16_t)> callback);
    bool sendTcpDataToClient(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no, const uint8_t* data, uint16_t length);
    bool closeTcpConnection(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no);

private:
    WifiLink link;
    ConsoleServer console;

    void on_beacon();
    SoftTimer beacon{"WifiBeacon", 1000, true, this, &WifiProvider::on_beacon};
    volatile bool beacon_due{false};
    void on_second_tick();
    void send_beacon(char* ip, char* netmask, u8 clients);

    void query_sta_param(Gcode *);
    void query_ap_param(Gcode *);
    bool config_ap(AP_PARAM_TYPE type, u8* value, u8 len, const char* what);
    void report_status(Gcode *);
    McodeRegistry::Mcode m482, m483, m489;

    static void shell(void* self, const char* cmd, std::string args, StreamOutput* stream);
    void sub_webserver(std::string args, StreamOutput* stream);
    void sub_tcp(std::string args, StreamOutput* stream);
    void sub_clients(std::string args, StreamOutput* stream);
    static const SimpleShell::Sub<WifiProvider> SUBS[];
    SimpleShell::Registered shell_slot;

    u16 list_status = 0;                // the module's last client list: 0 or why it failed
    u8 list_count = 0;

    void set_wifi_op_mode(u8 op_mode);
    void M8266WIFI_Module_Hardware_Reset(void);
    u8 M8266WIFI_Module_Init_Via_SPI();
    void init_wifi_module();
    void query_wifi_status();

    uint32_t ip_to_int(char* ip_addr);
    void int_to_ip(uint32_t i_ip, char *ip_addr);
    void get_broadcast_from_ip_and_netmask(char *broadcast_addr, char *ip_addr, char *netmask);

    uint8_t getNextLinkNo();
    uint8_t next_available_link_no;

    std::map<u8, std::function<void(u8*, u16, u8*, u16)>> data_callbacks;

    int connection_fail_count;
    char ap_address[16];
    char ap_netmask[16];
    char sta_address[16];
    char sta_netmask[16];
    char machine_name[32];
    uint16_t tcp_port, udp_send_port, udp_recv_port, tcp_timeout_s;

    u8 tcp_link_no;
    u8 udp_link_no;
    bool wifi_init_ok;
};

#endif /* WIFIPROVIDER_H_ */

extern WifiProvider wifi_provider;
