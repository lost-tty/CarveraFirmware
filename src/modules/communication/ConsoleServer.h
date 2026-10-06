#ifndef CONSOLESERVER_H
#define CONSOLESERVER_H

#include "Session.h"
#include "modules/utils/wifi/WifiLink.h"

#define MAX_SESSIONS 4                   // console clients at once, matching the module's own cap

class StreamOutput;

// The wifi console clients: a session for each, fed what the link received for the console
// link, reaped when the module no longer lists the client.
class ConsoleServer {
public:
    void attach(WifiLink* link, uint8_t link_no);

    // A received buffer of the console link: false while it waits at the head, for a transfer
    // to take with its next loan.
    bool take(WifiRxBuf& b);
    void reap(const ClientInfo* listed, u8 count);
    void pump();
    void list(StreamOutput* stream);

private:
    Session* session_for(const u8 ip[4], u16 port);

    Session sessions[MAX_SESSIONS];
    WifiLink* link = nullptr;
    uint8_t link_no = 0;
};

#endif
