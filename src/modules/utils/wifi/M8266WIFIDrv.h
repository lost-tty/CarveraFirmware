/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

// Driver for the M8266 SPI WiFi module, covering the calls the Carvera firmware makes.
// Wire-compatible with the vendor's V1.2.0-6 driver.
//
// Functions returning u8 give 0 on failure, otherwise 1 or the module's reply byte. A non-NULL
// status gets the error code in its low byte and the module's status register in its high byte.
// A query's param holds 32 bytes for an SSID, 64 for a password and 16 for anything else; the
// value ends in a NUL when shorter. RemoteClients holds M8266WIFI_MAX_CLIENTS entries.

#ifndef M8266WIFIDRV_H
#define M8266WIFIDRV_H

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

#define M8266WIFI_MAX_CLIENTS 15

#ifdef __cplusplus
extern "C" {
#endif

// Supplied by the host; the driver uses only the first four. delay_ms may sleep: the driver
// waits through it for anything of a millisecond or more, and between status polls once a
// quick answer is unlikely.
void M8266HostIf_Set_SPI_nCS_Pin(u8 level);
void M8266HostIf_delay_us(u8 nus);
void M8266HostIf_delay_ms(u16 nms);
u32 M8266HostIf_now_us(void);
void M8266HostIf_Set_nRESET_Pin(u8 level);

typedef enum {
    STA_PARAM_TYPE_SSID = 0,
    STA_PARAM_TYPE_PASSWORD = 1,
    STA_PARAM_TYPE_CHANNEL = 2,
    STA_PARAM_TYPE_HOSTNAME = 3,
    STA_PARAM_TYPE_USE_BSSID = 4,
    STA_PARAM_TYPE_BSSID = 5,
    STA_PARAM_TYPE_RSSI = 6,
    STA_PARAM_TYPE_IP_ADDR = 7,
    STA_PARAM_TYPE_GATEWAY_ADDR = 8,
    STA_PARAM_TYPE_NETMASK_ADDR = 9,
    STA_PARAM_TYPE_DHCPC = 10,
    STA_PARAM_TYPE_MAC = 11,
} STA_PARAM_TYPE;

typedef enum {
    AP_PARAM_TYPE_SSID = 0,
    AP_PARAM_TYPE_PASSWORD = 1,
    AP_PARAM_TYPE_CHANNEL = 2,
    AP_PARAM_TYPE_AUTHMODE = 3,
    AP_PARAM_TYPE_SSID_HIDDEN = 4,
    AP_PARAM_TYPE_MAX_CONNECT = 5,
    AP_PARAM_TYPE_BEACON_INTERVAL = 6,
    AP_PARAM_TYPE_IP_ADDR = 7,
    AP_PARAM_TYPE_GATEWAY_ADDR = 8,
    AP_PARAM_TYPE_NETMASK_ADDR = 9,
    AP_PARAM_TYPE_PHY_MODE = 10,
    AP_PARAM_TYPE_MAC = 11,
} AP_PARAM_TYPE;

struct ScannedSigs {
    char ssid[32];
    u8 channel;
    u8 authmode;
    s8 rssi;
};

typedef struct {
    u8 connection_state;
    u8 remote_ip[4];
    u16 remote_port;
} ClientInfo;

// Link and module setup
u8 M8266HostIf_SPI_Select(uint32_t spi_base_addr, uint32_t spi_clock, u16* status);
u8 M8266WIFI_SPI_Interface_Communication_OK(u8* byte);
u32 M8266WIFI_SPI_Interface_Communication_Stress_Test(u32 max_times);
u8 M8266WIFI_SPI_Get_Module_Info(u32* module_id, u8* flash_size, char* fw_ver, u16* status);
u8 M8266WIFI_SPI_Get_Opmode(u8* op_mode, u16* status);
u8 M8266WIFI_SPI_Set_Opmode(u8 op_mode, u8 saved, u16* status);
u8 M8266WIFI_SPI_Set_Tx_Max_Power(u8 tx_max_power, u16* status);

// Station
u8 M8266WIFI_SPI_STA_Connect_Ap(u8 ssid[32], u8 password[64], u8 saved, u8 timeout_in_s,
                                u16* status);
u8 M8266WIFI_SPI_STA_DisConnect_Ap(u16* status);
u8 M8266WIFI_SPI_Get_STA_Connection_Status(u8* connection_status, u16* status);
u8 M8266WIFI_SPI_Get_STA_IP_Addr(char* sta_ip, u16* status);
u8 M8266WIFI_SPI_STA_ScanSignals(struct ScannedSigs scanned_signals[], u8 max_signals,
                                 u8 channel, u8 show_hidden, u8 passive_not_active_scan,
                                 u32 channel_scan_time_ms_max, u32 channel_scan_time_ms_min,
                                 u8 timeout_in_s, u16* status);
#define M8266WIFI_SPI_STA_Scan_Signals(sigs, max_signals, channel, timeout_in_s, status) \
    M8266WIFI_SPI_STA_ScanSignals(sigs, max_signals, channel, 0, 0, 0, 0, timeout_in_s, status)
u8 M8266WIFI_SPI_STA_Fetch_Last_Scanned_Signals(struct ScannedSigs scanned_signals[],
                                                u8 max_signals, u16* status);
u8 M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE param_type, u8* param, u8* param_len,
                                 u16* status);
u8 M8266WIFI_SPI_Get_STA_Hostname(char hostname[28 + 1], u16* status);
u8 M8266WIFI_SPI_Set_STA_Hostname(char hostname[28 + 1], u16* status);

// Access point
u8 M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE param_type, u8* param, u8* param_len,
                                u16* status);
u8 M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE param_type, u8* param, u8 param_len, u8 saved,
                                 u16* status);

// Connections
u8 M8266WIFI_SPI_Setup_Connection(u8 tcp_udp, u16 local_port, char* remote_addr,
                                  u16 remote_port, u8 link_no, u8 timeout_in_s, u16* status);
u8 M8266WIFI_SPI_Delete_Connection(u8 link_no, u16* status);
u8 M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(u8 link_no, u16 timeout_in_s, u16* status);
u8 M8266WIFI_SPI_Config_Max_Clients_Allowed_To_A_Tcp_Server(u8 server_link_no, u8 max_allowed,
                                                            u16* status);
u8 M8266WIFI_SPI_List_Clients_On_A_TCP_Server(u8 server_link_no, u8* clients,
                                              ClientInfo RemoteClients[], u16* status);
u8 M8266WIFI_SPI_Disconnect_TcpClient(u8 link_no, ClientInfo* client_info, u16* status);

// Data
u16 M8266WIFI_SPI_Send_Udp_Data(u8 Data[], u16 Data_len, u8 link_no, char* udp_dest_addr,
                                u16 udp_dest_port, u16* status);
u16 M8266WIFI_SPI_Send_Data_to_TcpClient(u8 Data[], u16 Data_len, u8 server_link_no,
                                         char* tcp_client_dest_addr,
                                         u16 tcp_client_dest_port, u16* status);
// As Send_Data_to_TcpClient, but each piece of up to 32 bytes comes from give() just before it
// goes out: the bytes [at, at + n) of the Data_len sent. A piece not picked up is asked for
// again by the next send of the rest.
typedef void (*M8266_TxGive)(void* ctx, u16 at, u8* piece, u8 n);
u16 M8266WIFI_SPI_Send_to_TcpClient_from(M8266_TxGive give, void* ctx, u16 Data_len,
                                         u8 server_link_no, char* tcp_client_dest_addr,
                                         u16 tcp_client_dest_port, u16* status);
u8 M8266WIFI_SPI_Has_DataReceived(void);
// Picks where a receive's n bytes go, once its head has named the sender; ip is 0.0.0.0 and
// port 0 on modules up to 1.1.6-5. NULL: each chunk (32 bytes at most) goes to the take
// function as it is read instead.
typedef u8* (*M8266_RxDest)(void* ctx, u8 link_no, const u8 remote_ip[4], u16 remote_port, u16 n);
typedef void (*M8266_RxTake)(void* ctx, const u8* chunk, u16 n);
u16 M8266WIFI_SPI_RecvData_ex(u8 Data[], u16 max_len, uint16_t max_wait_in_ms, u8* link_no,
                              u8 remote_ip[4], u16* remote_port, u16* status);
u8 M8266WIFI_SPI_Query_Tcp_Window_num(u8 link_no, u8* tcp_wnd_num, u16* status);
u8 M8266WIFI_SPI_Config_Tcp_Window_num(u8 link_no, u8 tcp_wnd_num, u16* status);
u16 M8266WIFI_SPI_RecvData_to(M8266_RxDest where, M8266_RxTake take, void* ctx, u16 max_len,
                              uint16_t max_wait_in_ms, u8* link_no, u8 remote_ip[4],
                              u16* remote_port, u16* status);

// The module's built-in web page
u8 M8266WIFI_SPI_Set_WebServer(u8 open_not_shutdown, u16 server_port, u8 saved, u16* status);
u8 M8266WIFI_SPI_Query_WebServer(u8* start_on_bootup, u8* current_running, u16* default_port,
                                 u16* current_port, u16* status);

#ifdef __cplusplus
}
#endif

#endif
