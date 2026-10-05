#include "Scripts.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "utils/Gcode.h"
#include "checksumm.h"
#include "ScriptsPublicAccess.h"
#include "modules/tools/atc/ATCHandler.h"
#include "utils/Parameters.h"
#include "SimpleShell.h"
#include "utils.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include "modules/robot/MachineTask.h"
#include "Program.h"
#include "MacroFS.h"


void Scripts::on_module_loaded()
{
    gcode_dispatch.set_script_hook(this);
    SimpleShell::add_command(shell_slot, "macro", &Scripts::shell, this,
                             "macro list | params | run <sub> [args]");
    load();
}

static const std::vector<std::string> MACRO_PATHS= {SCRIPTS_DIR, "/" MACROFS_MOUNT "/"};

void Scripts::load()
{
    program.macros().load(MACRO_PATHS);
    printk("scripts: %u\n", unsigned(program.macros().names().size()));
}

// M6 T3 -> o<m6> with #<t> = 3, G28.2 -> o<g28.2>: every word of the block becomes a #<letter>.
// A code without a sub stays with its C++ handler. The sub is only pushed here, its first line runs
// on the next main loop; stream gets the ok when it is done.
bool Scripts::trigger(const Gcode &gcode, StreamOutput *stream, std::string &err)
{
    // while a sub runs, only its own lines reach the dispatcher: they keep the codes' built-in
    // meaning instead of triggering the sub they came from
    if(program.in_sub())
        return false;

    const gcode::Word &c= gcode.command;
    if(c.letter == 0)
        return false;

    char sub[16];
    int n= snprintf(sub, sizeof(sub), "%c%u", c.letter, unsigned(c.value));
    if(c.subcode != 0)
        snprintf(sub + n, sizeof(sub) - n, ".%u", c.subcode);

    if(!program.macros().has(sub))
        return false;

    if(!program.call(sub, nullptr, 0, stream, err))
        return true;

    for (const gcode::Word &w : gcode.get_words()) {
        char local[2]= {(char)tolower(w.letter), 0};
        if(w.letter != 'G' && w.letter != 'M') program.set_local(local, w.value);
    }
    return true;
}

bool Scripts::call(const std::string &line, StreamOutput *stream, std::string &err)
{
    return program.call_line(line, stream, err);
}

void Scripts::file_changed(const char *path)
{
    if(path == nullptr)
        return;

    for (const std::string &dir : MACRO_PATHS) {
        if(strncmp(path, dir.c_str(), dir.size()) == 0) {
            load();
            return;
        }
    }
}

bool Scripts::run_sub(const char *sub, const float *args, unsigned nargs)
{
    if(!program.macros().has(sub))
        return false;

    std::string err;
    if(program.call(sub, args, nargs, nullptr, err))
        return true;

    printk("error:script %s %s\n", sub, err.c_str());
    return false;
}

// only queues the sub, it runs once the main loop is going
void Scripts::boot()
{
    run_sub("boot", nullptr, 0);
}

const SimpleShell::Sub<Scripts> Scripts::SUBS[] = {
    {"list",   &Scripts::sub_list,   "the subs that are defined"},
    {"params", &Scripts::sub_params, "the #<_name> values a script can read"},
    {"run",    &Scripts::sub_run,    "run one sub: run <sub> [args]"},
    {nullptr, nullptr, nullptr},
};

void Scripts::shell(void *self, const char *cmd, std::string args, StreamOutput *stream)
{
    SimpleShell::dispatch(static_cast<Scripts *>(self), SUBS, cmd, args, stream);
}

void Scripts::sub_list(std::string, StreamOutput *stream)
{
    for (const std::string &name : program.macros().names()) {
        stream->printf("%s\n", name.c_str());
    }
}

void Scripts::sub_params(std::string, StreamOutput *stream)
{
    Parameters::list_named(stream);
}

void Scripts::sub_run(std::string cmd, StreamOutput *stream)
{
    std::string sub= shift_parameter(cmd);
    float args[script::Runner::MAX_ARGS];
    unsigned n= 0;
    while(!cmd.empty() && n < script::Runner::MAX_ARGS) args[n++]= strtof(shift_parameter(cmd).c_str(), nullptr);
    if(machine_task.is_halted()) {
        stream->printf("error:Alarm lock\n");
        return;
    }
    std::string err;
    if(!program.call(sub.c_str(), args, n, stream, err))
        stream->printf("error:%s\n", err.c_str());

    // ok follows when the script has finished
}
