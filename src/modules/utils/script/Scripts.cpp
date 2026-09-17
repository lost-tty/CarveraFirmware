#include "Scripts.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "utils/Gcode.h"
#include "checksumm.h"
#include "PublicData.h"
#include "ScriptsPublicAccess.h"
#include "PublicDataRequest.h"
#include "utils.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

extern const char _binary_macros_ngc_start[], _binary_macros_ngc_end[]; // src/macros/*.ngc, concatenated by the makefile

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
    register_for_event(ON_CONSOLE_LINE_RECEIVED);
    register_for_event(ON_SET_PUBLIC_DATA);
    gcode_dispatch.set_script_hook(this);
    load(&THEKERNEL->streams);
}

#define SD_DIR "/sd/macros/"

bool Scripts::load(StreamOutput *stream)
{
    if(runner != nullptr && runner->running()) {
        stream->printf("error:script running\n");
        return false;
    }
    delete runner;
    runner= nullptr;
    Macros::Report r;
    std::string err;
    loaded= macros.load(_binary_macros_ngc_start, _binary_macros_ngc_end, SD_DIR, r, err);
    if(!r.fallback.empty()) stream->printf("error:%s, using the embedded scripts\n", r.fallback.c_str());
    if(!loaded) {
        stream->printf("error:%s\n", macros.located(err, macros.program().error_offset).c_str());
        return false;
    }
    runner= new script::Runner(macros.program(), gcode_dispatch.parameters());
    if(r.replaced + r.added > 0) stream->printf("scripts: %u embedded, %u replaced, %u added from " SD_DIR "\n", r.embedded, r.replaced, r.added);
    else stream->printf("scripts: %u embedded\n", r.embedded);
    return true;
}

// the lines around the one running, like a debugger
void Scripts::list(StreamOutput *stream, unsigned around)
{
    script::Source &src= macros.source();
    unsigned cur= runner->last_offset();
    int segment= src.segment_of(cur);
    if(segment < 0) return;
    unsigned at= src.segments[segment].base, first= at, n= 1, line= src.line_of(cur);
    while(n + around < line) { // the window starts `around` lines before the current one
        at= src.next(at);
        first= at;
        n++;
    }
    stream->printf("%s:\r\n", src.name(segment).c_str());
    std::string text;
    unsigned next;
    for (unsigned end= src.segments[segment].base + src.segments[segment].size;
         first < end && n <= line + around && src.line_at(first, text, next); first= next, n++) {
        stream->printf("%c %5u  %s\r\n", first == cur ? '>' : ' ', n, text.c_str());
    }
    src.release();
}

bool Scripts::run(const char *sub, const float *args, unsigned nargs, StreamOutput *stream, std::string &err)
{
    if(!loaded) {
        err= "no scripts";
        return false;
    }
    if(runner->running()) {
        err= "script running";
        return false;
    }
    if(!runner->start(sub, args, nargs, err)) return false;
    name= sub;
    reply= stream;
    preamble= true;
    THEROBOT.push_state();
    sources.push(this);
    return true;
}

void Scripts::finish()
{
    THEROBOT.pop_state();
    THEKERNEL->set_atc_state(ATC_NONE);
    reply= nullptr;
}

void Scripts::halt(int reason)
{
    THEKERNEL->set_halt_reason(reason);
    THEKERNEL->call_event(ON_HALT, nullptr);
}

// M6 T3 -> o<tool_change> with #<t> = 3 and #<subcode> = 0; every word of the block becomes a #<letter>.
// The sub is only pushed here, its first line runs on the next main loop; stream gets the ok when it is done.
bool Scripts::trigger(const Gcode &gcode, StreamOutput *stream, std::string &err)
{
    if(!gcode.has_g && !gcode.has_m) return false;
    const Trigger *t= nullptr;
    for (const Trigger &e : TRIGGERS) {
        bool code= (e.letter == 'G') ? (gcode.has_g && gcode.g == e.code) : (gcode.has_m && gcode.m == e.code);
        if(code && (e.any_subcode || gcode.subcode == 0)) t= &e;
    }
    if(t == nullptr || !loaded || macros.program().find_sub(t->sub) < 0) return false; // not scripted, the C++ handler takes it
    if(!run(t->sub, nullptr, 0, stream, err)) return true;
    runner->set_local("code", t->code);
    runner->set_local("subcode", gcode.subcode);
    for (const gcode::Word &w : gcode.get_words()) {
        char local[2]= {(char)tolower(w.letter), 0};
        if(w.letter != 'G' && w.letter != 'M') runner->set_local(local, w.value);
    }
    return true;
}

Source::Result Scripts::next(SerialMessage &msg)
{
    if(preamble) { // scripts are written in mm, absolute
        preamble= false;
        msg.message= "G21 G90";
        return LINE;
    }

    std::string err;
    switch(runner->step(msg.message, err)) {
        case script::Runner::LINE:
            if(trace) {
                char buf[16];
                snprintf(buf, sizeof(buf), "line %u:", macros.source().line_of(runner->last_offset()));
                printk("%s> %s\n", macros.located(buf, runner->last_offset()).c_str(), msg.message.c_str());
            }
            msg.stream= &THEKERNEL->streams; // errors from the line must reach someone
            return LINE;
        case script::Runner::MESSAGE:
            printk("%s\n", msg.message.c_str());
            return WAIT;
        case script::Runner::DONE: {
            float reason= runner->aborted();
            StreamOutput *caller= reply;
            finish();
            if(reason != 0) {
                printk("error:script %s aborted (%d)\n", name.c_str(), (int)reason);
                halt(reason > 0 && reason < 255 ? (int)reason : SCRIPT);
            } else if(caller != nullptr) {
                caller->printf("ok\r\n");
            }
            return DONE;
        }
        case script::Runner::ERROR: {
            StreamOutput *caller= reply;
            finish();
            std::string where= macros.located(err, runner->error_offset());
            printk("error:script %s %s\n", name.c_str(), where.c_str());
            if(caller != nullptr && caller != &THEKERNEL->streams) caller->printf("error:script %s %s\n", name.c_str(), where.c_str());
            halt(SCRIPT);
            return DONE;
        }
    }
    return WAIT;
}

void Scripts::abort()
{
    if(!runner->running()) return;
    runner->stop();
    if(reply != nullptr) reply->printf("error:script %s stopped\r\n", name.c_str());
    finish();
}

// hooks from other modules: run a sub if the machine script has it
void Scripts::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr= static_cast<PublicDataRequest *>(argument);
    if(!pdr->starts_with(scripts_checksum) || !pdr->second_element_is(run_script_checksum)) return;
    const script_call *c= static_cast<const script_call *>(pdr->get_data_ptr());
    if(!loaded || macros.program().find_sub(c->sub) < 0) return;
    std::string err;
    if(run(c->sub, c->args, c->nargs, nullptr, err)) pdr->set_taken();
    else printk("error:script %s %s\n", c->sub, err.c_str());
}

void Scripts::on_console_line_received(void *argument)
{
    SerialMessage *msg= static_cast<SerialMessage *>(argument);
    std::string cmd= msg->message;
    if(shift_parameter(cmd) != "macro") return;
    StreamOutput *stream= msg->stream;
    std::string what= shift_parameter(cmd);

    if(what == "check" || what == "reload") {
        if(load(stream)) stream->printf("ok\n");
    } else if(what == "list") {
        if(!loaded) {
            stream->printf("error:no scripts, try macro check\n");
            return;
        }
        const script::Program &p= macros.program();
        for (const script::Control &c : p.controls) {
            if(c.kind == script::SUB) stream->printf("%s\n", p.label_text(p.labels[c.label]).c_str());
        }
        stream->printf("ok\n");
    } else if(what == "trace") {
        trace= shift_parameter(cmd) == "on";
        stream->printf("ok\n");
    } else if(what == "run") {
        std::string sub= shift_parameter(cmd);
        float args[script::Runner::MAX_ARGS];
        unsigned n= 0;
        while(!cmd.empty() && n < script::Runner::MAX_ARGS) args[n++]= strtof(shift_parameter(cmd).c_str(), nullptr);
        std::string err;
        if(THEKERNEL->is_halted()) stream->printf("error:Alarm lock\n");
        else if(!run(sub.c_str(), args, n, stream, err)) stream->printf("error:%s\n", err.c_str());
        // ok follows when the script has finished
    } else {
        stream->printf("macro list | check | run <sub> [args] | trace on|off\n");
    }
}
