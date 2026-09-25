#include "Source.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "GcodeDispatch.h"
#include "SimpleShell.h"
#include "utils.h"

#include <algorithm>
#include <cstdlib>
#include "modules/robot/MachineTask.h"

void SourceStack::on_module_loaded()
{
    register_for_event(ON_MAIN_LOOP);
    SimpleShell::add_command(shell_slot, "list", &SourceStack::shell, this, "list [n] - lines around the one running");
}

void SourceStack::shell(void *self, const char *, std::string cmd, StreamOutput *stream)
{
    SourceStack *me= static_cast<SourceStack *>(self);
    std::string n= shift_parameter(cmd);
    unsigned around= n.empty() ? 10 : strtoul(n.c_str(), nullptr, 10);
    if(me->stack.empty()) stream->printf("Nothing running\r\n");
    for (Source *s : me->stack) s->list(stream, around);
}

bool SourceStack::push(Source *s)
{
    if(std::find(stack.begin(), stack.end(), s) != stack.end()) return false;
    stack.push_back(s);
    return true;
}

bool SourceStack::frozen_all() const
{
    return suspended() && stack.size() <= frozen;
}

void SourceStack::on_main_loop(void *)
{
    if(stack.empty() || machine_task.is_halted() || frozen_all()) return;

    if(machine_task.full()) return;

    if(stopping) {
        if(!machine_task.motion_passed(stop_after)) return;   // the job is over, it just has to finish moving
        stopping= false;
        printk("job stopped at line %u\n", stop_line);
        clear();
        return;
    }

    Source *s= stack.back();
    SerialMessage msg{&StreamOutput::NullStream, "", 0};
    switch(s->next(msg)) {
        case Source::LINE:
            // a halt inside clears the stack, s is not touched after this
            if(!gcode_dispatch.run_line(msg) && !stack.empty()) {
                if(stop_after_queued(msg.line)) break;
                printk("job stopped at line %u\n", msg.line);
                clear();
            }
            break;
        case Source::DONE:
            if(!stack.empty() && stack.back() == s) stack.pop_back();
            break;
        case Source::WAIT:
            break;
    }
}

void SourceStack::cleanup()
{
    clear();
}

void SourceStack::clear()
{
    while(!stack.empty()) {
        Source *s= stack.back();
        stack.pop_back();
        s->abort();
    }
    frozen= 0;
    stopping= false;
}

// false: nothing is queued ahead of it, so the caller stops the job itself
bool SourceStack::stop_after_queued(unsigned int line)
{
    uint32_t mark= machine_task.motion_mark();
    if(machine_task.motion_passed(mark)) return false;
    if(stopping) return true;   // the first refusal is the one that stopped the job

    stop_after= mark;
    stop_line= line;
    stopping= true;
    return true;
}

// the hold brakes on the path and keeps the queue, so resuming carries on from where it stood
void SourceStack::suspend()
{
    frozen= stack.size();
    machine_task.hold(true);
}

void SourceStack::resume()
{
    frozen= 0;
    machine_task.hold(false);
}
