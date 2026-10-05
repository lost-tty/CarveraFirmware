#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "GcodeDispatch.h"

#include <string>

// Loads the machine scripts and hands the G/M codes their subs take over to the program module.
class Scripts : public Module, public ScriptHook {
public:
    void on_module_loaded() override;
    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static const SimpleShell::Sub<Scripts> SUBS[];
    void sub_list(std::string args, StreamOutput *stream);
    void sub_params(std::string args, StreamOutput *stream);
    void sub_run(std::string args, StreamOutput *stream);
    void sub_trace(std::string args, StreamOutput *stream);
    SimpleShell::Registered shell_slot;
    bool trigger(const Gcode &gcode, StreamOutput *stream, std::string &err) override;
    bool call(const std::string &line, StreamOutput *stream, std::string &err) override;
    void boot();
    void file_changed(const char *path);
    bool run_sub(const char *sub, const float *args, unsigned nargs);

private:
    void load();
};

extern Scripts scripts;
