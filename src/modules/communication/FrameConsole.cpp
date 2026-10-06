#include "FrameConsole.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "libs/MainWake.h"
#include "SimpleShell.h"

#include <string>

uint16_t FrameConsole::feed(const uint8_t *p, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (!decoder.feed(p[i])) continue;
        on_frame();
        // A FILE_START starts an upload once its line runs, and the client sends the first of
        // its frames with it (the MD5 frame): the rest waits for the upload to take it.
        if (decoder.type() == Frame::FILE_START)
            return i + 1;
    }
    return len;
}

void FrameConsole::on_frame()
{
    console_frame(decoder.type(), decoder.payload(), decoder.length());
}

// A frame of this client's, from the decoder or found by an upload in its own buffer: a
// transfer's to the transfer, a key acted on, a line queued.
void FrameConsole::console_frame(uint8_t type, const uint8_t *p, uint16_t len)
{
    if (type >= Frame::FILE_MD5 && type <= Frame::FILE_RETRY) {
        SimpleShell::transfer_frame(this, type, p, len);
        return;
    }
    if (type != Frame::CTRL_SINGLE) {
        queue_frame(type, p, len);
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

void FrameConsole::pump()
{
    std::string line;
    if (!next_line(line))
        return;

    SimpleShell::run(line, this);
    wake_main();
}

void FrameConsole::queue_frame(uint8_t type, const uint8_t *p, uint16_t len)
{
    switch (type) {
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
