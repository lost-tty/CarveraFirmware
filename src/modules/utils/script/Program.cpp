#include "Program.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "modules/tools/atc/ATCHandler.h"
#include "modules/robot/MachineTask.h"
#include "modules/robot/Conveyor.h"
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

bool Program::load_job(const std::string &path, std::string &err)
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
    pause= JOB;
    waiting= true;
    return true;
}

bool Program::can_call(std::string &err) const
{
    if(holding()) {
        err= gcode_dispatch.parameters().behind() ? "a line waits for the machine"
                                                    : "paused before a line, step or resume first";
        return false;
    }
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
    follow_fence();
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

unsigned Program::played_line()
{
    uint32_t at[script::Runner::MAX_DEPTH];
    unsigned n= chain(machine_task.where().mark, at);
    if(at[n - 1] != 0)
        played= at[n - 1];

    return played;
}

unsigned Program::heads(Head *out)
{
    uint32_t at[script::Runner::MAX_DEPTH];
    unsigned depth= shown_chain(at), n= 0;
    for (unsigned i= depth; i-- > 0;) {
        // the outermost is the job's when one plays, even before its first line
        bool job= i == depth - 1 && playing();
        int segment= job ? source.job() : file_of(at[i]);
        if(segment < 0)
            continue;

        const script::Source::Segment &s= source.segments[segment];
        unsigned line= line_of(at[i]);
        out[n++]= Head{s.path, s.size, line, job ? runner->main().line - 1 : line, job,
                       job ? 0 : frame_of(at[i])};
    }
    return n;
}

unsigned Program::chain(uint32_t mark, uint32_t *out) const
{
    unsigned n= 0;
    out[n++]= mark;
    while(n < script::Runner::MAX_DEPTH && call_of(out[n - 1]) != 0) {
        out[n]= calls[call_of(out[n - 1]) % CALLS].from;
        n++;
    }
    return n;
}

uint32_t Program::standing() const
{
    if(holding())
        return stepper.held.mark;

    Conveyor::Fenced f= THECONVEYOR.fenced();
    return f.any ? f.mark : machine_task.where().mark;
}

unsigned Program::shown_chain(uint32_t *out)
{
    unsigned n= chain(standing(), out);
    if(out[n - 1] == 0 && playing())
        out[n - 1]= played_line();

    return n;
}

int Program::frame_of(uint32_t at) const
{
    for (unsigned level= 0; level < runner->depth(); level++) {
        uint32_t entered= runner->entered(level);
        if(entered != 0 && (entered & 0xFF) == call_of(at))
            return level;
    }
    return -1;
}

int Program::file_of(uint32_t at) const
{
    int segment= call_of(at) == 0 ? source.job() : calls[call_of(at) % CALLS].segment;
    return at != 0 && segment >= 0 && unsigned(segment) < source.segments.size() ? segment : -1;
}

std::string Program::file_line(int segment, unsigned line) const
{
    if(segment < 0)
        return "";

    char buf[16];
    snprintf(buf, sizeof(buf), ":%u", line);
    return source.basename(segment) + buf;
}

std::string Program::place(uint32_t at) const
{
    return file_line(file_of(at), line_of(at));
}

void Program::note_calls()
{
    for (unsigned level= 0; level < runner->depth(); level++) {
        uint32_t entered= runner->entered(level);
        Call &c= calls[entered % CALLS];
        if(entered == 0 || c.entered == entered)
            continue;

        c.entered= entered;
        c.segment= source.segment_of(runner->origin(level));
        // the caller's next line follows its call
        c.from= level > 0 ? mark_of(level - 1, runner->next_line(level - 1) - 1) : 0;
    }
}

void Program::park(const SerialMessage &msg)
{
    stepper.held= msg;
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
    std::string p= place(at);
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

    if(holding()) {
        msg= stepper.held;
        stepper.held.mark= 0;
        return true;
    }

    if(!runner->running()) {
        if(machine_task.idle()) end_job();
        return false;
    }

    std::string err;
    script::Runner::Result result= runner->step(msg.message, err);
    note_calls();
    switch(result) {
        case script::Runner::LINE: {
            msg.mark= mark_of(runner->depth() - 1, runner->last().line);
            if(trace)
                printk("%s> %s\n", place(msg.mark).c_str(), msg.message.c_str());

            msg.params= &runner->parameters();
            if(stops_before(runner->depth())) {
                stepper.held= msg;
                pause= ALL;
                return false;
            }
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
        case script::Runner::WAIT:
            return false;
        case script::Runner::ERROR: {
            std::string where= file_line(source.segment_of(runner->last().offset),
                                         runner->last().line) + ": " + err;
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
    pause= playing() ? JOB : ALL;
    machine_task.hold(true);
}

void Program::resume()
{
    lift_fence();
    stepper.stop_depth= 0;
    pause= NONE;
    release();
}

void Program::release()
{
    waiting= false;
    machine_task.hold(false);
}

void Program::lift_fence()
{
    if(!stepper.fencing)
        return;

    THECONVEYOR.ask_fence(Conveyor::FENCE_LIFT);
    stepper.fencing= false;
}

unsigned Program::depth_of(uint32_t mark) const
{
    uint32_t at[script::Runner::MAX_DEPTH];
    unsigned n= chain(mark, at);
    // without a job, a console call has no frame under it
    return at[n - 1] == 0 && !playing() ? n - 1 : n;
}

// lines queued before the suspend pass the conveyor's fence one by one
void Program::step(Step how)
{
    if(stepper.fencing && !THECONVEYOR.fence_settled())
        return;

    unsigned depth= depth_of(standing());
    if(stepper.fencing || (!holding() && !machine_task.idle())) {
        THECONVEYOR.ask_fence(stepper.fencing ? Conveyor::FENCE_PASS : Conveyor::FENCE_LINE);
        stepper.fencing= true;
        pause= ALL;
        release();
    } else {
        resume();
    }
    stepper.stop_depth= how == INTO ? EVERY : how == OVER ? depth : depth - 1;
}

void Program::follow_fence()
{
    if(!stepper.fencing || !THECONVEYOR.fence_settled())
        return;

    Conveyor::Fenced f= THECONVEYOR.fenced();
    if(f.any) {
        if(!stops_before(depth_of(f.mark)))
            THECONVEYOR.ask_fence(Conveyor::FENCE_PASS);

        return;
    }
    if(machine_task.work_pending())
        return;

    lift_fence();
    pause= NONE;
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
    if(!runner->goto_main(to, err))
        return false;

    // the stop that follows drops the queue and the fence with it
    stepper= Stepper{};
    return true;
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
    stepper= Stepper{};
    pause= NONE;
    waiting= false;
    stopping= false;
    gcode_dispatch.program_end();
}

void Program::cleanup()
{
    if(pause != NONE) {
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
    Outcome how= ending ? DONE : machine_task.is_halted() ? HALTED : STOPPED;
    ending= false;
    if(!playing())
        return;

    player.job_ended(how);
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
