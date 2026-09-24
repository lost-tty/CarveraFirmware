#include "StreamOutput.h"
#include "Kernel.h"
#include "StreamOutputPool.h"
#include "Frame.h"

NullStreamOutput StreamOutput::NullStream;
AllStreamsOutput StreamOutput::AllStreams;

int AllStreamsOutput::vprintf(const char *format, va_list args)
{
    return THEKERNEL->streams.vprintf(format, args);
}

int AllStreamsOutput::puts(const char *str, int size)
{
    return THEKERNEL->streams.puts(str, size);
}

void AllStreamsOutput::send(uint8_t type, const void *payload, size_t len)
{
    THEKERNEL->streams.send(type, payload, len);
}

// longer payloads are split into several frames of the same type
static const size_t MAX_FRAME_PAYLOAD = 512;
static uint8_t frame_buf[MAX_FRAME_PAYLOAD + Frame::OVERHEAD];

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

void StreamOutput::send(uint8_t type, const void *payload, size_t len)
{
    const uint8_t *p = static_cast<const uint8_t *>(payload);
    do {
        size_t n = len > MAX_FRAME_PAYLOAD ? MAX_FRAME_PAYLOAD : len;
        size_t total = Frame::encode(type, p, n, frame_buf);
        puts(reinterpret_cast<const char *>(frame_buf), total);
        p += n;
        len -= n;
    } while (len > 0);
}
