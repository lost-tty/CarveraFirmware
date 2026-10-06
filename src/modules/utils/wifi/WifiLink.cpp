#include "WifiLink.h"

#include "libs/DeferredWake.h"
#include "libs/MainWake.h"

#include "semphr.h"
#include "InterruptIn.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace {
    StaticSemaphore_t module_lock_store;
    SemaphoreHandle_t module_lock = xSemaphoreCreateMutexStatic(&module_lock_store);

    // given whenever a client took something: a writer waiting for room looks again
    StaticSemaphore_t tx_room_store;
    SemaphoreHandle_t tx_room = xSemaphoreCreateBinaryStatic(&tx_room_store);

    // One write at a time per client, from any task.
    void tx_claim(TxRing& q)
    {
        void* me = xTaskGetCurrentTaskHandle();
        for (;;) {
            taskENTER_CRITICAL();
            bool mine = q.writer == nullptr;
            if (mine)
                q.writer = me;

            taskEXIT_CRITICAL();
            if (mine)
                return;

            vTaskDelay(1);
        }
    }

    // all of it in the AHB SRAM: the main RAM has no room to spare
    #define AHB __attribute__((section("AHBSRAM")))
    // Worst case, from the image's call graph: a download's frame read as it goes out between
    // two receives, 464 bytes to the read of its chunk (run -> receive -> send_requested ->
    // send_peer -> send_locked -> the driver's send -> DownloadFrame::read), then 632 through
    // newlib's read, FATFileHandle::read, f_read and FatFS's chain lookups to
    // SDFileSystem::disk_write (488 alone), the calls through function pointers and virtuals
    // added up by hand; an upload's sink writing its frame comes to 972. + 32 for an interrupt's
    // frame, + 32 for a context switch, + 16 for the overflow check's pattern = 1176, with 232
    // bytes on top. The task never calls printk: its depth is open-ended.
    const uint16_t k_stack_words = 352;
    StackType_t task_stack[k_stack_words] AHB;
    StaticTask_t task_store AHB;
    WifiRxBuf rx_buf AHB;
    #undef AHB

    // The buffer's contents and rx_ready pass between the task and the main loop: neither
    // side's accesses to the one may move past its write of the other.
    inline void ordered()
    {
        __asm volatile("" ::: "memory");
    }

    void ip_text(char out[16], const u8 ip[4])
    {
        snprintf(out, 16, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    }
}

ModuleLock::ModuleLock()
{
    xSemaphoreTake(module_lock, portMAX_DELAY);
}

ModuleLock::~ModuleLock()
{
    xSemaphoreGive(module_lock);
}

void WifiLink::start(uint8_t console_link, mbed::InterruptIn* pin)
{
    this->console_link = console_link;
    this->pin = pin;
    if (pin != nullptr)
        pin->rise(this, &WifiLink::on_pin_rise);

    // the main loop's priority: it never spins long, so neither waits on the other
    task = xTaskCreateStatic(task_entry, "Wifi", k_stack_words, this, 1, task_stack, &task_store);
    defer_wake_to(WAKE_WIFI, task, 0);
}

// EINT3 runs above the syscall priority (the spindle's edges share it), so the wake goes
// through the RIT
void WifiLink::on_pin_rise()
{
    defer_wake(WAKE_WIFI);
}

void WifiLink::task_entry(void* self)
{
    static_cast<WifiLink*>(self)->run();
}

// Replies first, then what arrived. The pin wakes it when data arrives; it also looks every
// WIFI_POLL_MS for a module whose pin is not wired, and after a tick when a send found the
// module busy.
void WifiLink::run()
{
    for (;;) {
        {
            TickType_t before = xTaskGetTickCount();
            ulTaskNotifyTake(pdTRUE, tx_retry ? 1 : pdMS_TO_TICKS(WIFI_POLL_MS));
            // a wake already pending returns at once: that was no rest
            if (xTaskGetTickCount() != before)
                rested_at = xTaskGetTickCount();
        }
        rest_if_due();
        send_requested();
        receive();
    }
}

// Reads while the module has data and a buffer is free; with none free it leaves the data in
// the module and says so, and the main loop wakes it when it hands one back.
void WifiLink::receive()
{
    for (;;) {
        // the pin stays high while the module holds data; the status register is asked only
        // when it is low, for a module whose pin is not wired
        bool pending = pin != nullptr && pin->read();
        if (!pending) {
            ModuleLock lock;
            pending = M8266WIFI_SPI_Has_DataReceived();
        }
        if (!pending)
            return;

        // the sender is known only once a receive is under way: whoever it is, the shared
        // buffer must be free for it
        if (rx_ready) {
            rx_starved = true;
            return;
        }

        WifiRxBuf& b = rx_buf;
        u8 link = 0xFF;
        u16 status = 0;
        bool kept, sunk;
        {
            ModuleLock lock;
            // With a sink attached, ask past the shared buffer's size: the module then hands
            // over several segments at once. The sender is known only once the receive is
            // under way: other clients send command frames of at most 128 bytes, so more than
            // the shared buffer holds from one of them is rare, and dropped.
            u16 max = WIFI_DATA_MAX_SIZE;
            for (auto& s : sinks) {
                if (s.sink != nullptr)
                    max = WIFI_RX_SINK_MAX;
            }

            rx_landing = b.data;
            rx_sink = nullptr;
            rx_peer = nullptr;
            rx_spill = false;
            b.len = M8266WIFI_SPI_RecvData_to(pick_rx, take_rx, this, max, WIFI_DATA_TIMEOUT_MS,
                                              &link, b.ip, &b.port, &status);
            sunk = rx_sink != nullptr;
            if (sunk)
                answer_sink(b, link);


            // handed over before the lock goes: attach must see a peer's bytes once read
            kept = !sunk && !rx_spill && link != 0xFF && b.len != 0;
            if (kept) {
                b.link = link;
                b.off = 0;
                ordered();
                rx_ready = true;
                wake_main();
            }
        }
        if (kept || (sunk && b.len != 0)) {
            // an upload can keep the module busy for long: the others' output goes out between
            send_requested();
            rest_if_due();
            continue;
        }

        return;
    }
}

void WifiLink::rest_if_due()
{
    if (xTaskGetTickCount() - rested_at < pdMS_TO_TICKS(WIFI_REST_MS))
        return;

    vTaskDelay(1);
    rested_at = xTaskGetTickCount();
}

// Each client in turn, a piece at a time, until none takes more: one the module is busy for
// waits for the next look while the others are served.
void WifiLink::send_requested()
{
    tx_retry = false;
    for (bool moved = true; moved;) {
        moved = false;
        for (WifiPeer* p : tx_peers) {
            if (p != nullptr && send_peer(p))
                moved = true;
        }
    }
}

// The client's next piece: from its ring up to the download frame, if one waits, then that
// frame. Under the lock, so forget() finds no send under way. true: the client took something.
bool WifiLink::send_peer(WifiPeer* p)
{
    TxRing& q = p->tx;
    if (q.tail == q.head && q.source == nullptr)
        return false;

    ModuleLock lock;
    if (q.buf == nullptr || !p->live())
        return false;

    uint32_t head = q.head;
    TxSource* src = q.source;
    ordered();
    uint32_t end = src != nullptr ? q.source_at : head;
    char ip_str[16];
    ip_text(ip_str, p->who.ip);
    u16 status = 0;
    size_t len, n;
    if (q.tail != end) {
        uint32_t at = q.tail % WIFI_TX_RING;
        len = std::min<size_t>(std::min<size_t>(end - q.tail, WIFI_TX_RING - at),
                               WIFI_DATA_MAX_SIZE);
        n = send_locked(ip_str, p->who.port, p->link, q.buf, nullptr, at, len, &status);
        q.tail += n;
    } else if (src != nullptr) {
        len = std::min<size_t>(src->size() - q.source_sent, WIFI_DATA_MAX_SIZE);
        n = send_locked(ip_str, p->who.port, p->link, nullptr, src, q.source_sent, len, &status);
        q.source_sent += n;
        if (q.source_sent == src->size()) {
            q.source = nullptr;
            src->busy = false;
            // a download waits for its frame to have gone
            wake_main();
        }
    } else {
        return false;
    }
    p->tx_bytes += n;
    TickType_t now = xTaskGetTickCount();
    if (n != 0) {
        q.moved_at = now;
        xSemaphoreGive(tx_room);
    }
    if (n < len) {
        u8 err = status & 0xff;
        bool busy = err == 0x11 || err == 0x12;
        if (busy && now - q.moved_at < pdMS_TO_TICKS(WIFI_TX_STALE_MS))
            tx_retry = true;
        else
            discard(p);
    }
    return n != 0;
}

// What waits for the client, gone: a client that is gone, or takes nothing.
void WifiLink::discard(WifiPeer* p)
{
    TxRing& q = p->tx;
    q.tail = q.head;
    q.moved_at = xTaskGetTickCount();
    if (TxSource* src = q.source) {
        q.source = nullptr;
        src->busy = false;
        wake_main();
    }
    xSemaphoreGive(tx_room);
}

void WifiLink::forget(WifiPeer* p)
{
    ModuleLock lock;
    discard(p);
}

// A client the task serves: its ring, on its first write. false: no memory, or no slot.
bool WifiLink::serve(WifiPeer* p)
{
    if (p->tx.buf == nullptr) {
        uint8_t* buf = (uint8_t*)malloc(WIFI_TX_RING);
        if (buf == nullptr)
            return false;

        p->tx.buf = buf;
    }
    taskENTER_CRITICAL();
    WifiPeer* volatile* free_slot = nullptr;
    bool known = false;
    for (auto& slot : tx_peers) {
        if (slot == p)
            known = true;
        else if (slot == nullptr && free_slot == nullptr)
            free_slot = &slot;
    }
    if (!known && free_slot != nullptr)
        *free_slot = p;

    taskEXIT_CRITICAL();
    return known || free_slot != nullptr;
}

WifiRxBuf* WifiLink::rx_head()
{
    if (!rx_ready)
        return nullptr;

    ordered();
    return &rx_buf;
}

void WifiLink::rx_release()
{
    if (!rx_ready)
        return;

    ordered();
    rx_ready = false;
    wake_starved();
}

void WifiLink::wake_starved()
{
    if (rx_starved) {
        rx_starved = false;
        xTaskNotifyGive(task);
    }
}

// Under the lock no receive is under way. The peer's bytes still in the shared buffer came
// first: the sink takes them now, and what the task receives for it after.
bool WifiLink::attach(WifiPeer* p, RxSink* sink)
{
    if (task == nullptr)
        return false;

    ModuleLock lock;
    // the peer's slot, else a free one
    decltype(&sinks[0]) slot = nullptr;
    for (auto& s : sinks) {
        if (s.peer == p) {
            slot = &s;
            break;
        }
        if (slot == nullptr && s.sink == nullptr)
            slot = &s;
    }
    if (slot == nullptr)
        return false;

    slot->peer = p;
    slot->sink = sink;
    if (WifiRxBuf* h = rx_head()) {
        WifiRxBuf& b = *h;
        if (b.link == console_link && p->is(b.ip, b.port) && b.off < b.len) {
            p->rx_bytes += b.len - b.off;
            sink->take(b.data + b.off, b.len - b.off);
            b.off = b.len;
            rx_release();
        }
    }
    return true;
}

// Under the lock no receive is under way: nothing reaches the sink after this.
void WifiLink::detach(WifiPeer* p)
{
    ModuleLock lock;
    for (auto& s : sinks) {
        if (s.peer == p) {
            s.peer = nullptr;
            s.sink = nullptr;
        }
    }
}

bool WifiLink::has_sink(const WifiPeer* p) const
{
    for (auto& s : sinks) {
        if (s.peer == p && s.sink != nullptr)
            return true;
    }
    return false;
}

u8* WifiLink::pick_rx(void* self, u8 link, const u8 ip[4], u16 port, u16 n)
{
    WifiLink* w = static_cast<WifiLink*>(self);
    if (link == w->console_link) {
        for (auto& s : w->sinks) {
            if (s.sink != nullptr && s.peer->is(ip, port)) {
                w->rx_sink = s.sink;
                w->rx_peer = s.peer;
                s.peer->rx_bytes += n;
                return nullptr;
            }
        }
    }
    // asked past the shared buffer for a sink, and another client had more than it holds
    w->rx_spill = n > WIFI_DATA_MAX_SIZE;
    return w->rx_spill ? nullptr : w->rx_landing;
}

void WifiLink::take_rx(void* self, const u8* chunk, u16 n)
{
    WifiLink* w = static_cast<WifiLink*>(self);
    if (w->rx_sink != nullptr)
        w->rx_sink->take(chunk, n);
}

// A writer waits for room only while its client takes something: one that does not read
// costs it WIFI_TX_STUCK_MS once, then its frames are dropped at once.
int WifiLink::write(WifiPeer* s, const uint8_t* data, size_t len)
{
    if (task == nullptr || xTaskGetCurrentTaskHandle() == task || len == 0)
        return 0;

    TxRing& q = s->tx;
    tx_claim(q);
    bool queued = false;
    if (len <= WIFI_TX_RING && serve(s)) {
        for (;;) {
            uint32_t head = q.head, used = head - q.tail;
            if (used == 0)
                q.moved_at = xTaskGetTickCount();

            if (WIFI_TX_RING - used >= len) {
                uint32_t at = head % WIFI_TX_RING;
                size_t first = std::min<size_t>(len, WIFI_TX_RING - at);
                memcpy(q.buf + at, data, first);
                memcpy(q.buf, data + first, len - first);
                ordered();
                q.head = head + len;
                queued = true;
                break;
            }
            if (xTaskGetTickCount() - q.moved_at >= pdMS_TO_TICKS(WIFI_TX_STUCK_MS))
                break;

            // the timeout covers a give the task made before this writer began to wait
            xTaskNotifyGive(task);
            xSemaphoreTake(tx_room, pdMS_TO_TICKS(10));
        }
    }
    if (!queued)
        q.dropped = q.dropped + 1;

    q.writer = nullptr;
    xTaskNotifyGive(task);
    return queued ? (int)len : 0;
}

bool WifiLink::write_source(WifiPeer* s, TxSource* src)
{
    if (task == nullptr || xTaskGetCurrentTaskHandle() == task)
        return false;

    TxRing& q = s->tx;
    tx_claim(q);
    bool queued = q.source == nullptr && serve(s);
    if (queued) {
        q.source_at = q.head;
        q.source_sent = 0;
        src->busy = true;
        ordered();
        q.source = src;
    }
    q.writer = nullptr;
    xTaskNotifyGive(task);
    return queued;
}

// Under the receive's lock, so the sink is not detached meanwhile: its reply goes out at once,
// past what is queued for the peer, then the sink's slow part.
void WifiLink::answer_sink(const WifiRxBuf& b, u8 link)
{
    uint8_t out[16];
    size_t n = rx_sink->reply(out, sizeof(out));
    if (n != 0) {
        char ip_str[16];
        ip_text(ip_str, b.ip);
        u16 status = 0;
        rx_peer->tx_bytes += send_locked(ip_str, b.port, link, out, nullptr, 0, n, &status);
    }
    rx_sink->settle();
}

namespace {
    struct Reading {
        TxSource* src;
        size_t at;
    };
}

void WifiLink::give(void* from, u16 at, u8* piece, u8 n)
{
    Reading* r = (Reading*)from;
    r->src->read(r->at + at, piece, n);
}

// One send of at most WIFI_DATA_MAX_SIZE, ModuleLock held.
u16 WifiLink::send_locked(const char* ip_str, u16 port, u8 link, const u8* data, TxSource* src,
                          size_t at, u16 len, u16* status)
{
    char* addr = const_cast<char*>(ip_str);
    if (src == nullptr)
        return M8266WIFI_SPI_Send_Data_to_TcpClient((u8*)(data + at), len, link, addr, port,
                                                    status);

    Reading from = { src, at };
    return M8266WIFI_SPI_Send_to_TcpClient_from(give, &from, len, link, addr, port, status);
}

u16 WifiLink::send_to_client(const u8 ip[4], u16 port, u8 link, const u8* data, size_t len)
{
    char ip_str[16];
    ip_text(ip_str, ip);
    TickType_t start = xTaskGetTickCount();
    size_t done = 0;
    while (done < len) {
        u16 to_send = std::min<size_t>(len - done, WIFI_DATA_MAX_SIZE);
        u16 status = 0;
        u16 sent = locked([&] {
            return send_locked(ip_str, port, link, data, nullptr, done, to_send, &status);
        });
        done += sent;
        if (sent == to_send)
            continue;

        // 0x11 (waiting for wifi to send) and 0x12 (send buffer full) mean busy: the module
        // drains by itself, so resend the remainder. Anything else is a dead client, give up.
        u8 err = status & 0xFF;
        if ((err != 0x11 && err != 0x12)
            || xTaskGetTickCount() - start >= pdMS_TO_TICKS(WIFI_TX_STALE_MS))
            break;

        vTaskDelay(1);
    }
    return done;
}
