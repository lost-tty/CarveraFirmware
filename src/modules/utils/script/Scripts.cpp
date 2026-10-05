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


// G/M codes a script may take over, by defining the sub in the machine script
struct Trigger { char letter; uint16_t code; bool any_subcode; const char *sub; };
static const Trigger TRIGGERS[]= {
    {'M',   6, true,  "tool_change"},
    {'M', 321, false, "laser_on"},
    {'M', 322, false, "laser_off"},
    {'M', 491, true,  "calibrate"},
    {'M', 495, true,  "auto_work"},
    {'M', 496, true,  "goto"},
    {'G',  28, false, "g28"},
    {'G',  80, false, "drill_cancel"},
    {'G',  81, false, "drill"},
    {'G',  82, false, "drill"},
    {'G',  83, false, "drill"},
    {'G',  98, false, "drill_retract_z"},
    {'G',  99, false, "drill_retract_r"},
};

void Scripts::on_module_loaded()
{
    gcode_dispatch.set_script_hook(this);
    SimpleShell::add_command(shell_slot, "macro", &Scripts::shell, this,
                             "macro list | params | check | run <sub> [args] | trace on|off");
    register_for_event(ON_MAIN_LOOP);
    load();
}

#define SD_DIR SCRIPTS_DIR
bool Scripts::load()
{
    if(program.busy()) {
        printk("error:script running\n");
        return false;
    }
    Macros &macros= program.macros();
    Macros::Report r;
    std::string err;
    loaded= macros.load(Macros::EMBEDDED_DIR, SD_DIR, r, err);
    if(!r.fallback.empty()) printk("error:%s, using the embedded scripts\n", r.fallback.c_str());
    if(!loaded) {
        printk("error:%s\n", macros.located(err, macros.program().error_offset).c_str());
        return false;
    }
    if(r.replaced + r.added > 0) printk("scripts: %u embedded, %u replaced, %u added from " SD_DIR "\n", r.embedded, r.replaced, r.added);
    else printk("scripts: %u embedded\n", r.embedded);
    return true;
}

bool Scripts::run(const char *sub, const float *args, unsigned nargs, StreamOutput *stream, std::string &err)
{
    if(!loaded) {
        err= "no scripts";
        return false;
    }
    return program.call(sub, args, nargs, stream, err);
}

// M6 T3 -> o<tool_change> with #<t> = 3 and #<subcode> = 0; every word of the block becomes a #<letter>.
// The sub is only pushed here, its first line runs on the next main loop; stream gets the ok when it is done.
bool Scripts::trigger(const Gcode &gcode, StreamOutput *stream, std::string &err)
{
    // while a sub runs, only its own lines reach the dispatcher: they keep the codes' built-in
    // meaning instead of triggering the sub they came from
    if(program.in_sub())
        return false;

    if(!gcode.has_g() && !gcode.has_m()) return false;
    const Trigger *t= nullptr;
    for (const Trigger &e : TRIGGERS) {
        bool code= (e.letter == 'G') ? (gcode.has_g() && gcode.g() == e.code) : (gcode.has_m() && gcode.m() == e.code);
        if(code && (e.any_subcode || gcode.subcode() == 0)) t= &e;
    }
    // not scripted, the C++ handler takes it
    if(t == nullptr || !loaded || program.macros().program().find_sub(t->sub) < 0)
        return false;

    if(!run(t->sub, nullptr, 0, stream, err)) return true;
    // a canned cycle's sub moves in its own group 1; the program's is put back when the sub ends
    if(t->letter == 'G' && t->code >= 80 && t->code <= 89) program.restore_modal_on_return();
    program.set_local("code", t->code);
    program.set_local("subcode", gcode.subcode());
    for (const gcode::Word &w : gcode.get_words()) {
        char local[2]= {(char)tolower(w.letter), 0};
        if(w.letter != 'G' && w.letter != 'M') program.set_local(local, w.value);
    }
    return true;
}

bool Scripts::call(const std::string &line, StreamOutput *stream, std::string &err)
{
    if(!loaded) {
        err= "no scripts";
        return false;
    }
    return program.call_line(line, stream, err);
}

// hooks from other modules: run a sub if the machine script has it
// a macro file changed on disk, so the loaded program is stale
void Scripts::file_changed(const char *path)
{
    if(path == nullptr || strncmp(path, SD_DIR, sizeof(SD_DIR) - 1) != 0) return;
    if(program.busy()) { // the source still reads the job and the subs: reload once they are done
        stale= true;
        return;
    }
    loaded= false;
    load();
}

void Scripts::on_main_loop(void *)
{
    if(!stale || program.busy())
        return;

    stale= false;
    file_changed(SD_DIR);
}

bool Scripts::run_sub(const char *sub, const float *args, unsigned nargs)
{
    if(!loaded || program.macros().program().find_sub(sub) < 0)
        return false;

    std::string err;
    if(run(sub, args, nargs, nullptr, err)) return true;
    printk("error:script %s %s\n", sub, err.c_str());
    return false;
}

// only queues the sub, it runs once the main loop is going
void Scripts::boot()
{
    run_sub("boot", nullptr, 0);
}

const SimpleShell::Sub<Scripts> Scripts::SUBS[] = {
    {"check",  &Scripts::sub_check,  "reload the scripts and validate them"},
    {"list",   &Scripts::sub_list,   "the subs that are defined"},
    {"params", &Scripts::sub_params, "the #<_name> values a script can read"},
    {"run",    &Scripts::sub_run,    "run one sub: run <sub> [args]"},
    {"trace",  &Scripts::sub_trace,  "echo every executed line: trace on|off"},
    {nullptr, nullptr, nullptr},
};

void Scripts::shell(void *self, const char *cmd, std::string args, StreamOutput *stream)
{
    SimpleShell::dispatch(static_cast<Scripts *>(self), SUBS, cmd, args, stream);
}

void Scripts::sub_check(std::string, StreamOutput *)
{
    load();
}

void Scripts::sub_list(std::string, StreamOutput *stream)
{
    if(!loaded) {
        stream->printf("error:no scripts, try macro check\n");
        return;
    }
    const script::Program &p= program.macros().program();
    for (unsigned i= 0; i < p.subs.size(); i++) stream->printf("%s\n", p.name(i));
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
    std::string err;
    if(machine_task.is_halted()) stream->printf("error:Alarm lock\n");
    else if(!run(sub.c_str(), args, n, stream, err)) stream->printf("error:%s\n", err.c_str());
    // ok follows when the script has finished
}

void Scripts::sub_trace(std::string cmd, StreamOutput *stream)
{
    program.set_trace(shift_parameter(cmd) == "on");
}
