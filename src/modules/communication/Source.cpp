#include "Source.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "GcodeDispatch.h"
#include "SimpleShell.h"
#include "Conveyor.h"
#include "utils.h"

#include <algorithm>
#include <cstdlib>

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
    return THEKERNEL->is_suspending() && stack.size() <= frozen;
}

void SourceStack::on_main_loop(void *)
{
    if(stack.empty() || THEKERNEL->is_halted() || THEKERNEL->is_waiting() || frozen_all()) return;

    // a line refused while earlier moves are still queued stops the job once they have run
    unsigned int refused;
    if(THECONVEYOR.refusal_due(refused)) {
        printk("job stopped at line %u\n", refused);
        THECONVEYOR.flush_queue();
        clear();
        return;
    }

    if(THECONVEYOR.refused()) return;   // the job is over, it just has to finish moving

    Source *s= stack.back();
    SerialMessage msg{&StreamOutput::NullStream, "", 0};
    switch(s->next(msg)) {
        case Source::LINE:
            // a halt inside clears the stack, s is not touched after this
            if(!gcode_dispatch.run_line(msg) && !stack.empty()) {
                if(THECONVEYOR.refuse_after_queued(msg.line)) break;
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
}

void SourceStack::suspend()
{
    frozen= stack.size();
    THEKERNEL->set_suspending(true);
}

void SourceStack::resume()
{
    frozen= 0;
    THEKERNEL->set_suspending(false);
}
