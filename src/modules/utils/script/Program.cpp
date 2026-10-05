#include "Program.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "modules/tools/atc/ATCHandler.h"
#include "modules/robot/MachineTask.h"
#include "Player.h"
#include "MacroFS.h"
#include "ScriptsPublicAccess.h"
#include "utils/Parameters.h"
#include "utils.h"

#include <cstdio>
#include <cstdlib>

static const std::vector<std::string> MACRO_PATHS= {SCRIPTS_DIR, "/" MACROFS_MOUNT "/"};

void Program::on_module_loaded()
{
    files.load(MACRO_PATHS);
    printk("%u macros found\n", unsigned(files.names().size()));
    library.source= &source;
    library.resolver= &files;
    runner= new script::Runner(library, gcode_dispatch.parameters());
    SimpleShell::add_command(macro_slot, "macro", &Program::macro_shell, this,
                             "macro list | params");
    SimpleShell::add_command(trace_slot, "trace", &Program::trace_shell, this,
                             "trace on|off - echo every line with its file and number");
    SimpleShell::add_command(list_slot, "list", &Program::list_shell, this,
                             "list [n] - lines around the one running");
}

bool Program::remap(const char *sub, const gcode::Words &words, std::string &err)
{
    // a sub's own lines keep the codes' built-in meaning
    if(in_sub() || !files.has(sub))
        return false;

    if(call(sub, nullptr, 0, &THEKERNEL->streams, err)) {
        for (const gcode::Word &w : words) {
            char local[2]= {(char)tolower(w.letter), 0};
            if(w.letter != 'G' && w.letter != 'M')
                runner->set_local(local, w.value);
        }
    }
    return true;
}

bool Program::run_sub(const char *sub)
{
    if(!files.has(sub))
        return false;

    std::string err;
    if(call(sub, nullptr, 0, nullptr, err))
        return true;

    printk("error:script %s %s\n", sub, err.c_str());
    return false;
}

bool Program::start_job(const std::string &path, std::string &err)
{
    script::Source &src= source;
    library.reset();
    if(!src.add_job(path)) {
        err= "File not found: " + path;
        return false;
    }
    if(!runner->start_main(src.segments[src.job()].base, err)) {
        src.remove_job();
        return false;
    }
    name= path;
    played= 0;
    return true;
}

bool Program::can_call(std::string &err) const
{
    if(!in_sub())
        return true;

    err= "script running";
    return false;
}

bool Program::called(const std::string &sub, StreamOutput *stream, bool on_job)
{
    nested= on_job;
    name= sub;
    reply= stream;
    return true;
}

bool Program::call(const char *sub, const float *args, unsigned nargs, StreamOutput *stream,
                   std::string &err)
{
    bool on_job= runner->at_main();
    return can_call(err) && runner->call(sub, args, nargs, err) && called(sub, stream, on_job);
}

bool Program::call_line(const std::string &text, StreamOutput *stream, std::string &err)
{
    bool on_job= runner->at_main();
    std::string sub;
    return can_call(err) && runner->call_line(text, sub, err) && called(sub, stream, on_job);
}

bool Program::step(SerialMessage &msg)
{
    if(!busy() || machine_task.is_halted() || frozen())
        return false;

    if(!stopping)
        return advance(msg);

    // the job is over, it just has to finish moving
    if(!machine_task.motion_passed(stop_after))
        return false;

    stopping= false;
    stop_at(stop_mark);
    return false;
}

// in a sub: the last job line that ran
unsigned Program::played_line()
{
    uint32_t at= machine_task.where().mark;
    if(at != 0 && script::Source::segment(at) == 0)
        played= at;

    return played;
}

void Program::refused(uint32_t at)
{
    // a halt inside stops everything, so nothing is left to stop here
    if(!busy())
        return;

    if(stop_after_queued(at))
        return;

    stop_at(at);
}

void Program::stop_at(uint32_t at)
{
    std::string p= source.place(at);
    printk("job stopped at %s\n", p.empty() ? "a console line" : p.c_str());
    stop();
}

bool Program::advance(SerialMessage &msg)
{
    if(pause_asked && runner->at_main()) {
        pause_asked= false;
        suspend();
        printk("Suspended, resume to continue playing\n");
        return false;
    }

    if(!runner->running()) {
        if(machine_task.idle()) end_job();
        return false;
    }

    std::string err;
    switch(runner->step(msg.message, err)) {
        case script::Runner::LINE: {
            msg.mark= runner->mark();
            if(trace)
                printk("%s> %s\n", source.place(msg.mark).c_str(), msg.message.c_str());

            msg.params= &runner->parameters();
            return true;
        }
        case script::Runner::MESSAGE:
            printk("%s\n", msg.message.c_str());
            return false;
        case script::Runner::RETURNED:
            if(nested) {
                finish();
                name= job_name();
            }
            return false;
        case script::Runner::DONE: {
            float reason= runner->aborted();
            finish();
            ending= playing();
            if(reason != 0) {
                printk("error:script %s aborted (%d)\n", name.c_str(), (int)reason);
                halt(reason > 0 && reason < 255 ? (int)reason : SCRIPT);
            }
            return false;
        }
        case script::Runner::ERROR: {
            std::string where= source.place(runner->mark()) + ": " + err;
            finish();
            printk("error:script %s %s\n", name.c_str(), where.c_str());
            halt(SCRIPT);
            return false;
        }
    }
    return false;
}

const char *Program::job_name() const
{
    const script::Source &src= source;
    return playing() ? src.segments[src.job()].path : "";
}

unsigned Program::job_size() const
{
    const script::Source &src= source;
    return playing() ? src.segments[src.job()].size : 0;
}

unsigned Program::job_read() const
{
    if(!playing())
        return 0;

    const script::Source &src= source;
    return runner->main().offset - src.segments[src.job()].base;
}

// the hold brakes on the path and keeps the queue, so resuming carries on from where it stood
void Program::suspend()
{
    paused= true;
    machine_task.hold(true);
}

void Program::resume()
{
    paused= false;
    machine_task.hold(false);
}

bool Program::cancel_pause()
{
    if(!pause_asked)
        return false;

    pause_asked= false;
    return true;
}

bool Program::jump(unsigned to, std::string &err)
{
    return runner->goto_main(to, err);
}

// false: nothing is queued ahead of it, so the caller stops the job itself
bool Program::stop_after_queued(uint32_t at)
{
    uint32_t mark= machine_task.motion_mark();
    if(machine_task.motion_passed(mark))
        return false;

    if(stopping) // the first refusal is the one that stopped the job
        return true;

    stop_after= mark;
    stop_mark= at;
    stopping= true;
    return true;
}

void Program::stop()
{
    if(runner->running()) {
        runner->stop();
        if(reply != nullptr) reply->printf("error:script %s stopped\r\n", name.c_str());
        finish();
    }
    end_job();
    paused= false;
    stopping= false;
    gcode_dispatch.program_end();
}

void Program::cleanup()
{
    if(paused) {
        resume();
        printk("Suspend cleared\n");
    }
    stop();
}

// a sub returned or everything ended
void Program::finish()
{
    nested= false;
    machine_task.enforce_keepout();
    atc_handler.set_state(0);
    reply= nullptr;
    forget();
}

// an SD file may change before the next program
void Program::forget()
{
    if(!runner->running() && !playing())
        library.reset();
}

// the file ended, was stopped or the machine halted: queued motion finishes, then spindle and
// coolant go off as after M2
void Program::end_job()
{
    ending= false;
    if(!playing())
        return;

    player.job_ended();
    source.remove_job();
    pause_asked= false;
    machine_task.enforce_keepout();
    forget();
}

void Program::halt(int reason)
{
    machine_task.halt(reason, name.empty() ? "script aborted" : name.c_str());
}

void Program::trace_shell(void *self, const char *, std::string cmd, StreamOutput *stream)
{
    Program *me= static_cast<Program *>(self);
    std::string on= shift_parameter(cmd);
    if(on == "on" || on == "off") {
        me->trace= on == "on";
    } else if(!on.empty()) {
        stream->printf("error:trace on|off\r\n");
        return;
    }
    stream->printf("trace %s\r\n", me->trace ? "on" : "off");
}

void Program::list_shell(void *self, const char *, std::string cmd, StreamOutput *stream)
{
    Program *me= static_cast<Program *>(self);
    std::string n= shift_parameter(cmd);
    unsigned around= n.empty() ? 10 : strtoul(n.c_str(), nullptr, 10);
    if(!me->busy()) {
        stream->printf("Nothing running\r\n");
        return;
    }

    script::Source &src= me->source;
    if(me->playing()) {
        stream->printf("%s:\r\n", me->job_name());
        me->list(stream, src.job(), me->played_line(), around);
    }
    uint32_t at= machine_task.where().mark;
    unsigned segment= script::Source::segment(at);
    if(segment != 0 && segment < src.segments.size()) {
        stream->printf("%s:\r\n", src.basename(segment).c_str());
        me->list(stream, segment, script::Source::line(at), around);
    }
}

// the lines of a segment around the current one, like a debugger
void Program::list(StreamOutput *stream, int segment, unsigned current, unsigned around)
{
    if(segment < 0)
        return;

    script::Source &src= source;
    unsigned at= src.segments[segment].base, end= at + src.segments[segment].size, n= 1;
    for (; n + around < current && at < end; n++) at= src.next(at); // `around` lines before
    std::string text;
    for (unsigned next; at < end && n <= current + around && src.line_at(at, text, next);
         at= next, n++) {
        stream->printf("%c %5u  %s\r\n", n == current ? '>' : ' ', n, text.c_str());
    }
    src.release();
}

const SimpleShell::Sub<Program> Program::MACRO_SUBS[] = {
    {"list",   &Program::macro_list,   "the subs that are defined"},
    {"params", &Program::macro_params, "the #<_name> values a script can read"},
    {nullptr, nullptr, nullptr},
};

void Program::macro_shell(void *self, const char *cmd, std::string args, StreamOutput *stream)
{
    SimpleShell::dispatch(static_cast<Program *>(self), MACRO_SUBS, cmd, args, stream);
}

void Program::macro_list(std::string, StreamOutput *stream)
{
    for (const std::string &n : files.names()) {
        stream->printf("%s\n", n.c_str());
    }
}

void Program::macro_params(std::string, StreamOutput *stream)
{
    Parameters::list_named(stream);
}
