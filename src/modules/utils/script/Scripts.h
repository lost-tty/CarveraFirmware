#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "GcodeDispatch.h"
#include "Macros.h"
#include "Source.h"

#include <string>

// Runs O-word scripts. The machine scripts (src/macros/*.ngc, embedded; a file of the same name in
// /sd/macros/ replaces it, extra files add subs) provide subs that take over G/M codes when defined
// (M6 -> o<tool_change> ...) and can be run from the shell with "macro run <sub> [args]". A running script
// is a source on the stack: it pauses the job it was started from and feeds one line per main loop.
class Scripts : public Module, public ScriptHook, public Source {
public:
    void on_module_loaded() override;
    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static const SimpleShell::Sub<Scripts> SUBS[];
    void sub_check(std::string args, StreamOutput *stream);
    void sub_list(std::string args, StreamOutput *stream);
    void sub_params(std::string args, StreamOutput *stream);
    void sub_run(std::string args, StreamOutput *stream);
    void sub_trace(std::string args, StreamOutput *stream);
    SimpleShell::Registered shell_slot;
    void on_set_public_data(void *) override;
    bool trigger(const Gcode &gcode, StreamOutput *stream, std::string &err) override;
    Source::Result next(SerialMessage &msg) override;
    void abort() override;
    void list(StreamOutput *stream, unsigned around) override;
    void boot();

private:
    bool load(StreamOutput *stream);
    bool run(const char *sub, const float *args, unsigned nargs, StreamOutput *reply, std::string &err);
    void finish();
    void halt(int reason);

    Macros macros;
    script::Runner *runner= nullptr;
    StreamOutput *reply= nullptr;       // caller waiting for ok/error
    std::string name;                   // sub being run, for messages
    bool preamble= false;               // G21 G90 still to be sent before the first script line
    bool loaded= false;
    bool trace= false;                  // echo every executed line with its origin
};

extern Scripts scripts;
