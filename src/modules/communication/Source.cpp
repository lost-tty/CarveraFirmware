#include "Source.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "GcodeDispatch.h"
#include "utils.h"

#include <algorithm>
#include <cstdlib>

void SourceStack::on_module_loaded()
{
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_HALT);
    register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

void SourceStack::on_console_line_received(void *argument)
{
    SerialMessage *msg= static_cast<SerialMessage *>(argument);
    std::string cmd= msg->message;
    if(shift_parameter(cmd) != "list") return;
    std::string n= shift_parameter(cmd);
    unsigned around= n.empty() ? 10 : strtoul(n.c_str(), nullptr, 10);
    if(stack.empty()) msg->stream->printf("Nothing running\r\n");
    for (Source *s : stack) s->list(msg->stream, around);
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

    Source *s= stack.back();
    SerialMessage msg{&StreamOutput::NullStream, "", 0};
    switch(s->next(msg)) {
        case Source::LINE:
            gcode_dispatch.run_line(msg, s->internal()); // a halt inside clears the stack, s is not touched after this
            break;
        case Source::DONE:
            if(!stack.empty() && stack.back() == s) stack.pop_back();
            break;
        case Source::WAIT:
            break;
    }
}

void SourceStack::on_halt(void *argument)
{
    if(argument == nullptr) clear();
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
