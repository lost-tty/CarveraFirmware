#include "ConsoleServer.h"

#include "libs/Logging.h"
#include "libs/StreamOutput.h"

void ConsoleServer::attach(WifiLink* link, uint8_t link_no)
{
    this->link = link;
    this->link_no = link_no;
}

// A console's bytes go into its decoder. What follows a FILE_START waits at the head for the
// upload it starts, whose sink takes it first (WifiLink::attach).
bool ConsoleServer::take(WifiRxBuf& b)
{
    Session* s = session_for(b.ip, b.port);
    // no room; the module should have refused them
    if (s == nullptr)
        return true;

    s->fresh = true;
    uint16_t fed = s->feed(b.data + b.off, b.len - b.off);
    s->rx_bytes += fed;
    b.off += fed;
    return b.off >= b.len;
}

Session* ConsoleServer::session_for(const u8 ip[4], u16 port)
{
    Session* free_slot = nullptr;
    for (Session& s : sessions) {
        if (!s.live()) {
            if (free_slot == nullptr)
                free_slot = &s;

            continue;
        }
        if (s.is(ip, port))
            return &s;
    }
    if (free_slot == nullptr)
        return nullptr;

    free_slot->bind(link, link_no, ip, port);
    return free_slot;
}

void ConsoleServer::reap(const ClientInfo* listed, u8 count)
{
    for (Session& s : sessions) {
        if (!s.live())
            continue;

        if (s.fresh) {
            s.fresh = false;
            continue;
        }
        bool known = false;
        for (u8 i = 0; i < count && !known; i++)
            known = s.is(listed[i].remote_ip, listed[i].remote_port);

        if (!known)
            s.release();
    }
}

// every client gets its turn: one client's lines must not wait on another's traffic
void ConsoleServer::pump()
{
    for (Session& s : sessions) {
        if (s.live())
            s.pump();
    }
}

void ConsoleServer::list(StreamOutput* stream)
{
    for (int i = 0; i < MAX_SESSIONS; i++) {
        const Session& s = sessions[i];
        if (!s.live()) {
            stream->printf("%d: free\n", i);
            continue;
        }
        stream->printf("%d: %u.%u.%u.%u:%u in %lu out %lu queued %lu dropped %lu%s%s%s\n", i,
                       s.who.ip[0], s.who.ip[1], s.who.ip[2], s.who.ip[3], s.who.port,
                       (unsigned long)s.rx_bytes, (unsigned long)s.tx_bytes,
                       (unsigned long)(s.tx.head - s.tx.tail), (unsigned long)s.tx.dropped,
                       &s == stream ? " (this)" : "", link->has_sink(&s) ? " uploading" : "",
                       s.fresh ? " fresh" : "");
    }
}
