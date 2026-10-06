#include "ConsoleWatch.h"

#include "WifiProvider.h"
#include "Player.h"
#include "libs/Kernel.h"
#include "libs/Frame.h"
#include "libs/StringStream.h"
#include "task.h"

#include <algorithm>

static_assert(ConsoleWatch::WATCHERS == MAX_SESSIONS + 1, "the USB console and every session");

void ConsoleWatch::on_module_loaded()
{
    SimpleShell::add_command(shell_slot, "watch", &ConsoleWatch::shell, this,
                             "on|off - push the job, status and modal state as they change");
}

void ConsoleWatch::shell(void *self, const char *, std::string args, StreamOutput *stream)
{
    ConsoleWatch *w = static_cast<ConsoleWatch *>(self);
    std::string what = shift_parameter(args);
    if (what == "off") {
        w->remove(stream);
        stream->printf("watch off\r\n");
    } else if (what != "on") {
        stream->printf("error:watch on|off\r\n");
    } else if (w->add(stream)) {
        stream->printf("watch on\r\n");
    } else {
        stream->printf("error:%d consoles watch already\r\n", WATCHERS);
    }
}

bool ConsoleWatch::add(StreamOutput *stream)
{
    StreamOutput **slot = std::find(watchers, watchers + WATCHERS, stream);
    if (slot == watchers + WATCHERS)
        slot = std::find(watchers, watchers + WATCHERS, nullptr);

    if (slot == watchers + WATCHERS)
        return false;

    *slot = stream;
    TickType_t again = xTaskGetTickCount() - pdMS_TO_TICKS(AGAIN_MS);
    job.at = status.at = modal.at = again;
    due = true;
    push_timer.start();
    return true;
}

void ConsoleWatch::remove(StreamOutput *stream)
{
    std::replace(watchers, watchers + WATCHERS, stream, (StreamOutput *)nullptr);
    if (std::count(watchers, watchers + WATCHERS, nullptr) == WATCHERS)
        push_timer.stop();
}

void ConsoleWatch::service()
{
    if (!due)
        return;

    due = false;
    TickType_t now = xTaskGetTickCount();
    // main loop only; kept, so its string keeps its room from push to push
    static StringStream text;
    text.clear();
    player.job_status("", &text);
    push(Frame::JOB, job, text.getOutput(), now);

    push(Frame::STATUS, status, THEKERNEL->get_query_string(), now);

    text.clear();
    simpleshell.print_state(&text);
    push(Frame::MODAL, modal, text.getOutput(), now);
}

void ConsoleWatch::push(uint8_t type, Pushed &last, const std::string &text, TickType_t now)
{
    uint16_t crc = Frame::crc16(0, (const uint8_t *)text.data(), text.size());
    if (crc == last.crc && now - last.at < pdMS_TO_TICKS(AGAIN_MS))
        return;

    last.crc = crc;
    last.at = now;
    for (StreamOutput *s : watchers) {
        if (s != nullptr)
            s->send(type, text.data(), text.size());
    }
}
