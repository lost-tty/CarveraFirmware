#include "Session.h"

#include "modules/utils/wifi/WifiProvider.h"
#include "SimpleShell.h"
#include "ConsoleWatch.h"

#include <string>

void Session::bind(WifiProvider* provider, uint8_t link, const uint8_t ip[4], uint16_t port)
{
    SimpleShell::cancel_transfer(this);
    console_watch.remove(this);
    provider->drop_held(this);
    owner = provider;
    this->link = link;
    who = Endpoint(ip, port);
    set_transferring(false);
    drop_queued();
    set_cwd("/");
    fresh = true;
    stall = 0;
}

void Session::release()
{
    SimpleShell::cancel_transfer(this);
    console_watch.remove(this);
    if (owner) owner->drop_held(this);
    set_transferring(false);
    drop_queued();
    who = Endpoint();
    owner = nullptr;
    fresh = false;
    stall = 0;
}

bool Session::is(const uint8_t ip[4], uint16_t port) const
{
    return who == Endpoint(ip, port);
}

void Session::pump()
{
    while (act_key()) { }
    std::string line;
    if (next_line(line)) SimpleShell::run(line, this);
    if (owner) owner->flush_tx();
}

int Session::puts(const char* s, int size)
{
    if (!owner || !live()) return 0;
    size_t n = size == 0 ? strlen(s) : size;
    return owner->stage(this, (const uint8_t *)s, n);
}

void Session::set_transferring(bool f)
{
    if (owner) {
        if (f) owner->flush_tx(false);
        else owner->drop_held(this);
    }
    FrameConsole::set_transferring(f);
}

bool Session::ready()
{
    if (!owner) return false;
    bool more = true;
    while (more) {
        more = owner->read_chunk(false);
        if (owner->held_for(this)) return true;
    }
    return false;
}

int Session::gets(char** buf, int size)
{
    if (!owner) return 0;
    int n = 0;
    if (!owner->take_held(this, buf, &n)) return 0;
    return n;
}
