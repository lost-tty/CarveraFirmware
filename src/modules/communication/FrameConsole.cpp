#include "FrameConsole.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "SimpleShell.h"

#include <string>

void FrameConsole::run_bytes(const uint8_t *p, uint16_t len, bool dispatch)
{
    if (transferring) return;   // the bytes on the wire are a transfer's payload
    for (uint16_t i = 0; i < len; i++) {
        if (!decoder.feed(p[i])) continue;
        if (dispatch) on_frame(); else queue_frame();
        if (transferring) return;   // a frame may have started a transfer: the rest is payload
    }
}

void FrameConsole::on_frame()
{
    const uint8_t *p = decoder.payload();
    uint16_t len = decoder.length();

    if (decoder.type() != Frame::CTRL_SINGLE) {
        queue_frame();
        return;
    }
    if (len < 1) return;
    handle_key(p[0]);
}

void FrameConsole::handle_key(uint8_t c)
{
    std::string s;
    switch (c) {
        case '?':
            s = THEKERNEL->get_query_string();
            send(Frame::STATUS, s.data(), s.size());
            break;
        case '*':
            s = THEKERNEL->get_diagnose_string();
            send(Frame::DIAG, s.data(), s.size());
            break;
        default:
            SimpleShell::control_char(c, this);
            break;
    }
}

bool FrameConsole::act_key()
{
    uint8_t c;
    if (keys.size() == 0) return false;
    keys.pop_front(c);
    handle_key(c);
    return true;
}

void FrameConsole::queue_frame()
{
    const uint8_t *p = decoder.payload();
    uint16_t len = decoder.length();

    switch (decoder.type()) {
        case Frame::CTRL_SINGLE:
            if (len < 1 || keys.size() >= keys.capacity()) return;
            keys.push_back(p[0]);
            break;

        case Frame::CTRL_MULTI:
        case Frame::FILE_START: {
            if ((int)len + 1 > buffer.capacity() - buffer.size()) {
                printk("console: line dropped, buffer full\n");
                return;
            }
            char last = '\n';
            for (uint16_t i = 0; i < len; i++) {
                char c = p[i] == '\r' ? '\n' : p[i];
                if (c == '\n' && last == '\n') continue;
                buffer.push_back(c);
                last = c;
            }
            if (last != '\n') buffer.push_back('\n');
            break;
        }

        default:
            break;
    }
}

bool FrameConsole::next_line(std::string &line)
{
    bool complete = false;
    int index = buffer.tail;
    while (index != buffer.head) {
        if (buffer.buffer[index] == '\n') { complete = true; break; }
        index = buffer.next_block_index(index);
    }
    if (!complete) return false;
    line.clear();
    for (;;) {
        char c;
        buffer.pop_front(c);
        if (c == '\n') return true;
        line += c;
    }
}
