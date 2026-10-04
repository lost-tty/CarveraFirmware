#include "Program.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "modules/tools/atc/ATCHandler.h"
#include "modules/robot/MachineTask.h"
#include "Player.h"
#include "utils.h"

#include <cstdio>
#include <cstdlib>

void Program::on_module_loaded()
{
    runner= new script::Runner(library.program(), gcode_dispatch.parameters());
    register_for_event(ON_MAIN_LOOP);
    SimpleShell::add_command(shell_slot, "list", &Program::shell, this,
                             "list [n] - lines around the one running");
}

bool Program::start_job(const std::string &path, bool echo_lines, std::string &err)
{
    script::Source &src= library.source();
    if(!src.add_job(path)) {
        err= "File not found: " + path;
        return false;
    }
    if(!runner->start_main(src.segments[src.job()].base, err)) {
        src.remove_job();
        return false;
    }
    name= path;
    line= 0;
    echo= echo_lines;
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

void Program::restore_modal_on_return()
{
    saved_modal= gcode_dispatch.modal_state();
    modal_saved= true;
}

void Program::on_main_loop(void *)
{
    if(!busy() || machine_task.is_halted() || frozen() || machine_task.full())
        return;

    if(stopping) {
        // the job is over, it just has to finish moving
        if(!machine_task.motion_passed(stop_after))
            return;

        stopping= false;
        printk("job stopped at line %u\n", stop_line);
        stop();
        return;
    }

    SerialMessage msg{&THEKERNEL->streams, "", 0, nullptr};
    if(!next(msg))
        return;

    // a halt inside stops everything, so nothing is left to stop here
    if(!gcode_dispatch.run_line(msg) && busy()) {
        if(stop_after_queued(msg.line))
            return;

        printk("job stopped at line %u\n", msg.line);
        stop();
    }
}

// false: nothing to dispatch this loop
bool Program::next(SerialMessage &msg)
{
    if(pause_asked && runner->at_main()) {
        pause_asked= false;
        suspend();
        printk("Suspended, resume to continue playing\n");
        return false;
    }

    if(runner->at_main() && !inserted.empty()) {
        msg.message= inserted.front();
        inserted.pop_front();
        // an inserted line runs ahead of the next job line, so that is the line a stop here reports
        msg.line= line + 1;
        return true;
    }

    if(!runner->running()) {
        if(machine_task.idle()) end_job();
        return false;
    }

    std::string err;
    switch(runner->step(msg.message, err)) {
        case script::Runner::LINE:
            // a sub's lines belong to the job line that called it
            msg.line= runner->at_main() ? (line= runner->last().line) : line;
            if(runner->at_main()) {
                if(echo) printk("%u: %s\n", line, msg.message.c_str());
            } else if(trace) {
                char buf[16];
                snprintf(buf, sizeof(buf), "line %u:", runner->last().line);
                printk("%s> %s\n", library.located(buf, runner->last().offset).c_str(),
                       msg.message.c_str());
            }
            msg.params= &runner->parameters();
            return true;
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
            finish();
            std::string where= library.located(err, runner->last().offset);
            printk("error:script %s %s\n", name.c_str(), where.c_str());
            halt(SCRIPT);
            return false;
        }
    }
    return false;
}

const char *Program::job_name() const
{
    const script::Source &src= library.source();
    return playing() ? src.segments[src.job()].path : "";
}

unsigned Program::job_size() const
{
    const script::Source &src= library.source();
    return playing() ? src.segments[src.job()].size : 0;
}

unsigned Program::job_read() const
{
    if(!playing())
        return 0;

    const script::Source &src= library.source();
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

bool Program::insert(const std::string &text)
{
    if(inserted.size() >= INSERT_LIMIT)
        return false;

    inserted.push_back(text);
    return true;
}

bool Program::jump(unsigned to, std::string &err)
{
    if(!runner->goto_main(to, err))
        return false;

    line= to - 1;
    return true;
}

// false: nothing is queued ahead of it, so the caller stops the job itself
bool Program::stop_after_queued(unsigned int at)
{
    uint32_t mark= machine_task.motion_mark();
    if(machine_task.motion_passed(mark))
        return false;

    if(stopping) // the first refusal is the one that stopped the job
        return true;

    stop_after= mark;
    stop_line= at;
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
    if(modal_saved) gcode_dispatch.set_modal_state(saved_modal);
    modal_saved= false;
    nested= false;
    machine_task.enforce_keepout();
    atc_handler.set_state(0);
    reply= nullptr;
}

// the file ended, was stopped or the machine halted: queued motion finishes, then spindle and
// coolant go off as after M2
void Program::end_job()
{
    ending= false;
    if(!playing())
        return;

    player.job_ended();
    library.source().remove_job();
    line= 0;
    echo= false;
    pause_asked= false;
    inserted.clear();
    machine_task.enforce_keepout();
}

void Program::halt(int reason)
{
    machine_task.halt(reason, name.empty() ? "script aborted" : name.c_str());
}

void Program::shell(void *self, const char *, std::string cmd, StreamOutput *stream)
{
    Program *me= static_cast<Program *>(self);
    std::string n= shift_parameter(cmd);
    unsigned around= n.empty() ? 10 : strtoul(n.c_str(), nullptr, 10);
    if(!me->busy()) {
        stream->printf("Nothing running\r\n");
        return;
    }

    script::Source &src= me->library.source();
    if(me->playing()) {
        stream->printf("%s:\r\n", me->job_name());
        me->list(stream, src.job(), machine_task.where().line, around);
    }
    if(me->in_sub()) {
        int segment= src.segment_of(me->runner->last().offset);
        stream->printf("%s:\r\n", src.basename(segment).c_str());
        me->list(stream, segment, me->runner->last().line, around);
    }
}

// the lines of a segment around the current one, like a debugger
void Program::list(StreamOutput *stream, int segment, unsigned current, unsigned around)
{
    if(segment < 0)
        return;

    script::Source &src= library.source();
    unsigned at= src.segments[segment].base, end= at + src.segments[segment].size, n= 1;
    for (; n + around < current && at < end; n++) at= src.next(at); // `around` lines before
    std::string text;
    for (unsigned next; at < end && n <= current + around && src.line_at(at, text, next);
         at= next, n++) {
        stream->printf("%c %5u  %s\r\n", n == current ? '>' : ' ', n, text.c_str());
    }
    src.release();
}
