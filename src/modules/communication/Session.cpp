#include "Session.h"

#include "modules/utils/wifi/WifiLink.h"
#include "SimpleShell.h"
#include "ConsoleWatch.h"
#include "libs/Kernel.h"

#include <string>

void Session::bind(WifiLink* owner, uint8_t link, const uint8_t ip[4], uint16_t port)
{
    SimpleShell::cancel_transfer(this);
    console_watch.remove(this);
    // what still waits went to the slot's last client
    owner->forget(this);
    this->owner = owner;
    this->link = link;
    who = Endpoint(ip, port);
    rx_bytes = tx_bytes = 0;
    // the client gets broadcasts as a stream of the kernel's pool while it is connected
    THEKERNEL->streams.append_stream(this);
    drop_queued();
    set_cwd("/");
    fresh = true;
}

void Session::release()
{
    THEKERNEL->streams.remove_stream(this);
    SimpleShell::cancel_transfer(this);
    console_watch.remove(this);
    drop_queued();
    if (owner)
        owner->forget(this);

    who = Endpoint();
    owner = nullptr;
    fresh = false;
}

int Session::puts(const char* s, int size)
{
    if (!owner || !live())
        return 0;

    size_t n = size == 0 ? strlen(s) : size;
    return owner->write(this, (const uint8_t *)s, n);
}

void Session::puts_source(TxSource* s)
{
    if (!owner || !live())
        return;

    // a frame that cannot go now is asked for again by the client
    flush_gathered();
    owner->write_source(this, s);
}

bool Session::attach_sink(RxSink* s)
{
    return owner && owner->attach(this, s);
}

void Session::detach_sink()
{
    if (owner)
        owner->detach(this);
}
