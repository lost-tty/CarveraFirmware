/*
 * WifiProvider.h
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#ifndef WIFIPROVIDER_H_
#define WIFIPROVIDER_H_

using namespace std;
#include <vector>
#include <queue>
#include <map>
#include <functional>

#include "Pin.h"
#include "Module.h"
#include "SoftTimer.h"
#include "StreamOutput.h"
#include "libs/McodeRegistry.h"

#include "M8266WIFIDrv.h"
#include "libs/RingBuffer.h"
#include "modules/communication/Session.h"
#include "libs/Frame.h"

#define WIFI_DATA_MAX_SIZE 1460
#define WIFI_DATA_TIMEOUT_MS 10
#define WIFI_TX_RETRIES 50
#define WIFI_TX_RETRIES_SHORT 5
#define MAX_WLAN_SIGNALS 8
#define WIFI_TCP_WINDOW 8                // the console server's receive window, in segments
#define MAX_SESSIONS 4                   // console clients at once, matching the module's own cap

// A network to join, and how joining it went (the wlan shell command).
struct ap_conn_info {
    char ssid[32];
    char password[64];
    char ip_address[16];
    bool has_error;
    char error_info[64];
    bool disconnect;
};

class WifiProvider : public Module, public StreamOutput
{
public:
    std::string scan_wlans();
    void connect_ap(struct ap_conn_info *s);
    void set_ap_channel(uint8_t channel);
    void set_ap_ssid(const char *ssid);
    void set_ap_password(const char *password);
    void set_ap_enabled(bool on);
    void on_module_loaded();
    void on_second_tick(void* argument);
    void service();

    uint8_t initializeTcpServer(uint16_t local_port, uint8_t max_clients);
    void removeTcpServer(uint8_t link_no);
    void registerTcpDataCallback(uint8_t link_no, std::function<void(uint8_t*, uint16_t, uint8_t*, uint16_t)> callback);
    bool sendTcpDataToClient(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no, const uint8_t* data, uint16_t length);
    bool closeTcpConnection(const uint8_t* remote_ip, uint16_t remote_port, uint8_t link_no);
    int puts(const char*, int size = 0);

    u16 send_to_client(const u8 ip[4], u16 port, u8 link, const u8* data, size_t len,
                   int retries = WIFI_TX_RETRIES);

    bool read_chunk(bool dispatch);
    bool held_for(const Session* s) const { return rx_owner == s; }
    bool take_held(Session* s, char** buf, int* n);
    void drop_held(const Session* s) { if (rx_owner == s) { rx_owner = nullptr; rx_len = 0; } }

    int stage(Session* s, const uint8_t* data, size_t len);
    int flush_tx(bool patient = true);

private:
    int flush_unlocked(bool patient = true);   // caller holds tx_lock

public:
    void configure(const void *cfg);

private:
    void on_beacon() { beacon_due= true; }
    SoftTimer beacon{"WifiBeacon", 1000, true, this, &WifiProvider::on_beacon};
    volatile bool beacon_due{false};
    void query_sta_param(Gcode *);
    void query_ap_param(Gcode *);
    bool config_ap(AP_PARAM_TYPE type, u8* value, u8 len, const char* what);
    void report_status(Gcode *);

    McodeRegistry::Mcode m482, m483, m489;
    void set_wifi_op_mode(u8 op_mode);

    void M8266WIFI_Module_Hardware_Reset(void);
    u8 M8266WIFI_Module_Init_Via_SPI();

    void init_wifi_module();
    void query_wifi_status();

    uint32_t ip_to_int(char* ip_addr);
    void int_to_ip(uint32_t i_ip, char *ip_addr);
    void get_broadcast_from_ip_and_netmask(char *broadcast_addr, char *ip_addr, char *netmask);

    void on_pin_rise();

    Session* session_for(const u8 ip[4], u16 port, bool create);
    void reap_sessions(const ClientInfo* listed, u8 count);


    uint8_t getNextLinkNo();

    uint8_t next_available_link_no;

    mbed::InterruptIn *wifi_interrupt_pin; // Interrupt pin for measuring speed
    float probe_slow_rate;

    string test_buffer;

    Session sessions[MAX_SESSIONS];

    u8 rx_buf[WIFI_DATA_MAX_SIZE];
    Session* rx_owner = nullptr;   // the session the bytes in rx_buf belong to, if unread
    u16 rx_len = 0;

    char tx_buf[WIFI_DATA_MAX_SIZE];   // frames staged for one client, flushed as one packet
    size_t tx_len = 0;
    Session* tx_owner = nullptr;


    std::map<u8, std::function<void(u8*, u16, u8*, u16)>> data_callbacks;

	int connection_fail_count;
	char ap_address[16];
	char ap_netmask[16];
	char sta_address[16];
	char machine_name[32];
	uint16_t tcp_port, udp_send_port, udp_recv_port, tcp_timeout_s;
	char sta_netmask[16];

    struct {
    	u8  tcp_link_no;
    	u8  udp_link_no;
    	bool wifi_init_ok:1;
    	volatile bool has_data_flag:1;
    };

};

#endif /* WIFIPROVIDER_H_ */

extern WifiProvider wifi_provider;
