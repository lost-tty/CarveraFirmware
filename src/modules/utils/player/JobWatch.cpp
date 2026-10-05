#include "JobWatch.h"

#include "Player.h"
#include "WifiProvider.h"
#include "libs/Frame.h"
#include "libs/StreamOutput.h"
#include "task.h"

#include <algorithm>
#include <cstring>

static_assert(JobWatch::WATCHERS == MAX_SESSIONS + 1, "the USB console and every session");

class StatusText : public StreamOutput {
public:
    char text[512];
    size_t len = 0;
    int puts(const char *s, int size) override
    {
        size_t n = std::min(size == 0 ? strlen(s) : (size_t)size, sizeof(text) - len);
        memcpy(text + len, s, n);
        len += n;
        return n;
    }
    // printf sends its text as a frame; the push frames it once
    void send(uint8_t, const void *p, size_t n) override { puts((const char *)p, n); }
};

bool JobWatch::add(StreamOutput *stream)
{
    StreamOutput **slot = std::find(watchers, watchers + WATCHERS, stream);
    if (slot == watchers + WATCHERS)
        slot = std::find(watchers, watchers + WATCHERS, nullptr);

    if (slot == watchers + WATCHERS)
        return false;

    *slot = stream;
    pushed_at = xTaskGetTickCount() - pdMS_TO_TICKS(AGAIN_MS);
    return true;
}

void JobWatch::remove(StreamOutput *stream)
{
    std::replace(watchers, watchers + WATCHERS, stream, (StreamOutput *)nullptr);
}

void JobWatch::tick()
{
    if (std::count(watchers, watchers + WATCHERS, nullptr) == WATCHERS)
        return;

    TickType_t now = xTaskGetTickCount();
    if (now - checked_at < pdMS_TO_TICKS(PUSH_MS))
        return;

    checked_at = now;
    StatusText status;
    player.job_status("", &status);
    uint16_t crc = Frame::crc16(0, (const uint8_t *)status.text, status.len);
    if (crc == pushed_crc && now - pushed_at < pdMS_TO_TICKS(AGAIN_MS))
        return;

    pushed_crc = crc;
    pushed_at = now;
    for (StreamOutput *s : watchers) {
        if (s != nullptr)
            s->send(Frame::JOB, status.text, status.len);
    }
}
