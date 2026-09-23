#ifndef WIFIPUBLICACCESS_H
#define WIFIPUBLICACCESS_H

struct ap_conn_info {
    char ssid[32];
    char password[64];
    char ip_address[15];
    bool has_error;
    char error_info[64];
    bool disconnect;
};

#endif
