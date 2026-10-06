#pragma once

#include "libs/Module.h"
#include "SimpleShell.h"
#include "FreeRTOS.h"
#include "libs/MainWake.h"
#include "libs/SoftTimer.h"

#include <cstdint>
#include <string>

class StreamOutput;

class ConsoleWatch : public Module {
public:
    void on_module_loaded() override;
    void service();

    bool add(StreamOutput *stream);
    void remove(StreamOutput *stream);

    static const int WATCHERS = 5;

private:
    static const unsigned PUSH_MS = 40;
    static const unsigned AGAIN_MS = 1000;

    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);

    struct Pushed { uint16_t crc = 0; TickType_t at = 0; };
    void push(uint8_t type, Pushed &last, const std::string &text, TickType_t now);

    void push_due()
    {
        due = true;
        wake_main();
    }

    SoftTimer push_timer{"Watch", PUSH_MS, true, this, &ConsoleWatch::push_due};

    StreamOutput *watchers[WATCHERS] = {};
    Pushed job, status, modal;
    volatile bool due = false;
    SimpleShell::Registered shell_slot;
};

extern ConsoleWatch console_watch;
