#include "FrameConsole.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "SimpleShell.h"

#include <string>

void FrameConsole::on_frame()
{
    const uint8_t *p = decoder.payload();
    uint16_t len = decoder.length();

    switch (decoder.type()) {
        // ? and * answer in their own frame types here; the rest are the same on every console
        case Frame::CTRL_SINGLE: {
            if (len < 1) return;
            std::string s;
            switch (p[0]) {
                case '?':
                    s = THEKERNEL->get_query_string();
                    send(Frame::STATUS, s.data(), s.size());
                    break;
                case '*':
                    s = THEKERNEL->get_diagnose_string();
                    send(Frame::DIAG, s.data(), s.size());
                    break;
                default:
                    SimpleShell::control_char(p[0], this);
                    break;
            }
            break;
        }

        case Frame::CTRL_MULTI:
        case Frame::FILE_START: {
            if ((int)len + 1 > buffer.capacity() - buffer.size()) return;
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
