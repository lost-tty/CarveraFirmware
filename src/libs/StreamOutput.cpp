#include "StreamOutput.h"
#include "Frame.h"
#include "FreeRTOS.h"
#include "semphr.h"

static StaticSemaphore_t output_lock_store;
static SemaphoreHandle_t output_lock = xSemaphoreCreateMutexStatic(&output_lock_store);

static StaticSemaphore_t broadcast_lock_store;
static SemaphoreHandle_t broadcast_lock = xSemaphoreCreateMutexStatic(&broadcast_lock_store);

static std::string shared_cwd = "/";

const std::string &StreamOutput::cwd() const { return shared_cwd; }
void StreamOutput::set_cwd(const std::string &path) { shared_cwd = path; }

static void lock_output()
{
    xSemaphoreTake(output_lock, portMAX_DELAY);
}

static void unlock_output()
{
    xSemaphoreGive(output_lock);
}

void StreamOutput::console_lock() { lock_output(); }
void StreamOutput::console_unlock() { unlock_output(); }

void StreamOutput::lock_broadcast() { xSemaphoreTake(broadcast_lock, portMAX_DELAY); }
void StreamOutput::unlock_broadcast() { xSemaphoreGive(broadcast_lock); }

// longer payloads are split into several frames of the same type
static const size_t MAX_FRAME_PAYLOAD = 512;
static uint8_t frame_buf[MAX_FRAME_PAYLOAD + Frame::OVERHEAD];

// the gathered text sits at the payload offset of frame_buf, so encode() needs no copy
static uint8_t *const gather_buf = frame_buf + Frame::PAYLOAD_AT;
static StreamOutput *gather_to = nullptr;
static size_t gathered = 0;

// caller holds output_lock
static void send_gathered()
{
    if (gathered == 0)
        return;

    size_t total = Frame::encode(Frame::INFO, gather_buf, gathered, frame_buf);
    gathered = 0;
    gather_to->puts(reinterpret_cast<const char *>(frame_buf), total);
}

void StreamOutput::puts_source(TxSource *s)
{
    uint8_t piece[64];
    size_t len = s->size();
    lock_output();
    send_gathered();
    for (size_t at = 0; at < len; at += sizeof(piece)) {
        size_t n = len - at < sizeof(piece) ? len - at : sizeof(piece);
        s->read(at, piece, n);
        puts(reinterpret_cast<const char *>(piece), n);
    }
    unlock_output();
}

void StreamOutput::flush_gathered()
{
    lock_output();
    send_gathered();
    unlock_output();
}

StreamOutput::Gather::Gather(StreamOutput *s)
{
    lock_output();
    send_gathered();
    prev = gather_to;
    gather_to = s;
    unlock_output();
}

StreamOutput::Gather::~Gather()
{
    lock_output();
    send_gathered();
    gather_to = prev;
    unlock_output();
}

void StreamOutput::send(uint8_t type, const void *payload, size_t len)
{
    lock_output();

    const uint8_t *p = static_cast<const uint8_t *>(payload);
    if (type == Frame::INFO && gather_to == this && len <= MAX_FRAME_PAYLOAD) {
        if (gathered + len > MAX_FRAME_PAYLOAD)
            send_gathered();

        memcpy(gather_buf + gathered, p, len);
        gathered += len;
        unlock_output();
        return;
    }

    // send the gathered text before this output
    send_gathered();
    do {
        size_t n = len > MAX_FRAME_PAYLOAD ? MAX_FRAME_PAYLOAD : len;
        size_t total = Frame::encode(type, p, n, frame_buf);
        puts(reinterpret_cast<const char *>(frame_buf), total);
        p += n;
        len -= n;
    } while (len > 0);

    unlock_output();
}

int StreamOutput::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = vprintf(format, args);
    va_end(args);
    return result;
}

int StreamOutput::vprintf(const char *format, va_list args)
{
    char b[64];
    char *buffer;

    int size = vsnprintf(b, sizeof(b), format, args) + 1; // +1 for the terminating \0

    if (size <= static_cast<int>(sizeof(b))) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }

    send(Frame::INFO, buffer, size - 1);

    if (buffer != b) {
        delete[] buffer;
    }

    return size - 1;
}
