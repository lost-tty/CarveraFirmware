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

void StreamOutput::send(uint8_t type, const void *payload, size_t len)
{
    lock_output();

    const uint8_t *p = static_cast<const uint8_t *>(payload);
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
