/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "GcodeDispatch.h"

#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "libs/Settings.h"
#include "Robot.h"
#include "Conveyor.h"
#include "MachineTask.h"
#include "utils/Gcode.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "checksumm.h"
#include "Source.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

// RS274 execution order of the commands in one block
// Smoothie-specific M codes (OTHER_M) keep their traditional place after the motion
enum Rank : uint8_t { FEED_MODE, TOOL_CHANGE, SPINDLE, COOLANT, DWELL, PLANE, UNITS, CUTTER_COMP,
                      TOOL_OFFSET, WCS, PATH, STROKE, DISTANCE, RETRACT, NON_MODAL, MOTION, OTHER_M, STOP };

enum Flag : uint8_t {
    AXIS_WORDS = 1,   // takes the axis words itself, so it cannot share a block with a motion word
    NEEDS_HOMED = 2,  // probing, tool change and the ATC moves: refused on an unhomed machine
    WHEN_HALTED = 4,  // still runs while halted: status queries and things that turn stuff off
    DRAINS = 8,       // reads or sets where the machine is, so the queue has to be empty first
};

struct Class { uint8_t group; Rank rank; uint8_t flags; };  // group 0: may be combined freely

static bool is_command(const gcode::Word &w) { return w.letter == 'G' || w.letter == 'M'; }
static bool is_axis(char letter) { return strchr("XYZABC", letter) != nullptr; }
static bool is_move_word(char letter) { return strchr("XYZABCIJKRF", letter) != nullptr; }

static Class classify(const gcode::Word &w)
{
    unsigned n= w.value;
    if(w.letter == 'G') {
        if(n <= 3 || n == 73 || (n >= 80 && n <= 89)) return {1, MOTION, 0};
        if(n == 38) return {1, MOTION, uint8_t(NEEDS_HOMED | DRAINS)};
        switch(n) {
            case 4: return {0, DWELL, DRAINS};
            case 10: return {0, NON_MODAL, AXIS_WORDS};
            case 17: case 18: case 19: return {2, PLANE, 0};
            case 20: case 21: return {6, UNITS, 0};
            case 22: return {16, STROKE, AXIS_WORDS};
            case 23: return {16, STROKE, 0};
            case 28: return {0, NON_MODAL, uint8_t(AXIS_WORDS | DRAINS)};
            case 30: return {0, NON_MODAL, uint8_t(AXIS_WORDS | NEEDS_HOMED | DRAINS)};
            case 29: case 31: case 32: return {0, NON_MODAL, uint8_t(AXIS_WORDS | DRAINS)};
            case 40: case 41: case 42: return {7, CUTTER_COMP, 0};
            case 43: case 49: return {8, TOOL_OFFSET, 0};
            case 54: case 55: case 56: case 57: case 58: case 59: return {12, WCS, 0};
            case 61: case 64: return {13, PATH, 0};
            case 90: case 91: return {uint8_t(w.subcode == 0 ? 3 : 4), DISTANCE, 0};
            case 92: return {0, NON_MODAL, uint8_t((w.subcode == 0 ? AXIS_WORDS : 0) | DRAINS)};
            case 93: case 94: case 95: return {5, FEED_MODE, 0};
            case 98: case 99: return {10, RETRACT, 0};
        }
        return {0, NON_MODAL, 0};
    }
    switch(n) {
        case 0: case 1: case 60: return {14, STOP, 0};
        case 2: case 30: return {14, STOP, WHEN_HALTED};
        case 6: return {15, TOOL_CHANGE, NEEDS_HOMED};
        case 3: case 4: return {16, SPINDLE, 0};
        case 5: return {16, SPINDLE, WHEN_HALTED};
        case 7: case 8: return {0, COOLANT, 0};
        case 9: return {0, COOLANT, WHEN_HALTED};
        case 48: case 49: return {17, OTHER_M, 0};
        case 80: case 81: case 105: case 106: case 107: case 114: case 119: case 503: case 911:
            return {0, OTHER_M, WHEN_HALTED};
        case 491: case 495: case 496: return {0, OTHER_M, NEEDS_HOMED};
    }
    return {0, OTHER_M, 0};
}

// a mode setting: axis words next to it are a move in the modal motion (G90 X10), unlike settings M codes (M92 X80)
static bool is_modal_setting(Class c) {
    return c.rank != DWELL && c.rank != NON_MODAL && c.rank != OTHER_M && c.rank != STOP && c.rank != TOOL_CHANGE && c.rank != MOTION;
}

Module *GcodeDispatch::handlers = nullptr;

bool GcodeDispatch::run_mcode(Gcode &gcode, bool nested)
{
    const McodeRegistry::Mcode *m= McodeRegistry::find(gcode.m, gcode.subcode);
    if(m == nullptr) return false;

    uint8_t when= m->when & ~McodeRegistry::MID_JOB;

    if(when == McodeRegistry::IMMEDIATE) {
        m->handler(m->owner, &gcode);
        return true;
    }

    // an ACTION waits for the moves written before it, a BARRIER for all of them
    MachineTask::Job job= when == McodeRegistry::ACTION ? hold_or_run : run_barrier;

    if(machine_task.on_task()) {
        job(gcode, machine_task.proof());
        return true;
    }

    if(!machine_task.post(job, gcode)) gcode.error_text= "machine busy";
    return true;
}

// an empty queue has nothing to wait behind
void GcodeDispatch::hold_or_run(Gcode &gcode, OnMachine)
{
    const McodeRegistry::Mcode *m= McodeRegistry::find(gcode.m, gcode.subcode);
    if(m == nullptr) return;
    if(THECONVEYOR.hold_action(m, gcode)) return;
    m->handler(m->owner, &gcode);
}

void GcodeDispatch::run_barrier(Gcode &gcode, OnMachine on)
{
    const McodeRegistry::Mcode *m= McodeRegistry::find(gcode.m, gcode.subcode);
    if(m == nullptr) return;
    if(!THECONVEYOR.wait_for_idle()) return;
    m->handler(m->owner, &gcode);
}

void GcodeDispatch::broadcast(Gcode &gcode, OnMachine)
{
    for (Module *m = handlers; m != nullptr; m = m->next_gcode_handler) m->on_gcode_received(&gcode);

    // the posting line has already returned, so the error is reported from here
    if(gcode.error_text.empty()) return;
    printk("error:%s\n", gcode.error_text.c_str());
    if(!THECONVEYOR.refuse_after_queued(gcode.line)) sources.clear();
}

// G4 and G92 read or set where the machine is, so the queue has to run out first
void GcodeDispatch::broadcast_drained(Gcode &gcode, OnMachine on)
{
    if(!THECONVEYOR.wait_for_idle()) return;
    broadcast(gcode, on);
}

void GcodeDispatch::run_gcode(Gcode &gcode, uint8_t flags, bool nested)
{
    MachineTask::Job job= (flags & DRAINS) ? broadcast_drained : broadcast;

    if(!machine_task.on_task()) {
        if(!machine_task.post(job, gcode)) gcode.error_text= "machine busy";
        return;
    }

    job(gcode, machine_task.proof());
}

void GcodeDispatch::add_handler(Module *module)
{
    module->next_gcode_handler = handlers;
    handlers = module;
}

void GcodeDispatch::init()
{
    Parameters::init();
    modal_group_1= 0;
    homed_check= true;

    ADD_MCODE(m500, 500, IMMEDIATE, GcodeDispatch::report_settings);
    ADD_MCODE(m503, 503, BESIDE_JOB, GcodeDispatch::report_settings);
}

// no module writes the config, so M500 has always only reported, like M503
void GcodeDispatch::report_settings(Gcode *gcode)
{
    Settings::report_all(gcode->stream);
}

// an error goes to every console: a job's lines reply to the null stream
bool GcodeDispatch::fail(StreamOutput *, const char *msg)
{
    printk("error:%s\n", msg);
    return false;
}


bool GcodeDispatch::safe_while_running(const gcode::Words &words)
{
    bool found_one= false;
    for (const gcode::Word &w : words) {
        if(w.letter == 'G') return false;   // a G code moves the machine or changes how it moves
        if(w.letter != 'M') continue;

        const McodeRegistry::Mcode *m= McodeRegistry::find(w.value, w.subcode);
        if(m == nullptr || !(m->when & McodeRegistry::MID_JOB)) return false;
        found_one= true;
    }
    return found_one;
}

// an interleaved line would move the machine out of sequence; a suspended job is safe to jog
void GcodeDispatch::run_mdi(const SerialMessage &msg)
{
    if(sources.active()) {
        // without the parameters: reading #5021 drains the queue, and the letters decide this
        gcode::Line parsed;
        if(!parsed.parse(msg.message.c_str(), nullptr)) {
            msg.stream->printf("error:%s, and parameters are not read while a job runs\r\n", parsed.error_text().c_str());
            return;
        }
        if(!safe_while_running(parsed.words())) {
            msg.stream->printf("error:busy, a job or script is running\r\n");
            return;
        }
    }
    run_line(msg);
}

bool GcodeDispatch::run_line(const SerialMessage &msg, bool nested)
{
    return dispatch(msg, nested);
}

bool GcodeDispatch::run_line(const std::string &line, StreamOutput *stream, bool nested)
{
    return dispatch(SerialMessage{stream, line, 0}, nested);
}

bool GcodeDispatch::dispatch(const SerialMessage &msg, bool nested)
{
    const string &s= msg.message;

    size_t i= s.find_first_not_of(" \t");
    if(i == string::npos) {
        return true;
    }

    char c= s[i];
    if(c == '$' || islower((unsigned char)c)) return true; // simpleshell command

    size_t j= i;
    if(c == 'N') {
        j++;
        while(j < s.size() && isdigit(s[j])) j++;
        while(j < s.size() && s[j] == ' ') j++;
    }
    if(j < s.size() && s[j] == '#') return parameter_statement(s.c_str() + j, msg.stream);

    gcode::Line parsed; // local: modules may dispatch console lines while a line executes
    if(!parsed.parse(s.c_str() + i, &params)) return fail(msg.stream, parsed.error_text().c_str());
    return execute(parsed.words(), s.substr(i), msg.stream, msg.line, nested);
}

// "#n = expr" assigns, "#n" prints
bool GcodeDispatch::parameter_statement(const char *p, StreamOutput *stream)
{
    std::string err;
    if(strchr(p, '=') != nullptr) {
        if(!gcode::assign(p, params, err)) return fail(stream, err.c_str());
    } else {
        char *end;
        int n= strtol(p + 1, &end, 10);
        float v;
        if(end == p + 1) return fail(stream, "bad parameter number");
        if(params.get(n, v)) stream->printf("#%d = %.4f\r\n", n, v);
        else stream->printf("#%d not set\r\n", n);
    }
    return true;
}

// M999 is the only way out of a halt, so it is handled before the alarm lock refuses everything else
GcodeDispatch::Gate GcodeDispatch::allowed_while_halted(const gcode::Words &words, StreamOutput *stream)
{
    if(!machine_task.is_halted()) return PASS;

    for (const gcode::Word &w : words) {
        if(w.letter == 'M' && w.value == 999) {
            machine_task.unlock(stream);
            return HANDLED;
        }
    }
    for (const gcode::Word &w : words) {
        if(!is_command(w)) {
            if(!is_axis(w.letter)) continue;
            stream->printf("error:Alarm lock\n");
            return REFUSED;
        }
        if(classify(w).flags & WHEN_HALTED) continue;
        stream->printf("error:Alarm lock\n");
        return REFUSED;
    }
    return PASS;
}

GcodeDispatch::Gate GcodeDispatch::homed_enough(const gcode::Words &words, StreamOutput *stream)
{
    for (const gcode::Word &w : words) {
        if(w.letter == 'M' && (w.value == 887 || w.value == 888)) {
            homed_check= (w.value == 887);
            stream->printf("Homed check %s\n", homed_check ? "enabled" : "disabled");
            return HANDLED;
        }
        if(!homed_check || !is_command(w)) continue;
        if((classify(w).flags & NEEDS_HOMED) && !THEROBOT.is_homed_all_axes()) {
            fail(stream, "Machine has not been homed, home first (M888 disables this check)");
            return REFUSED;
        }
    }
    return PASS;
}

bool GcodeDispatch::execute(const gcode::Words &words, const string &text, StreamOutput *stream, unsigned int line, bool nested)
{
    if(words.empty()) {
        return true;
    }

    Gate gate= allowed_while_halted(words, stream);
    if(gate == PASS) gate= homed_enough(words, stream);
    if(gate != PASS) return gate == HANDLED;

    // A line holds one or more blocks: a repeated modal group, or a G53 after a motion word, starts
    // the next one (Smoothie lines like "G0 A90 G53 G0 Z-2"). Words belong to the block they appear in.
    struct Cmd { size_t index; uint8_t block; Rank rank; };
    struct Blk { bool mcs; bool motion; bool axis_code; bool axis; bool feed; bool settings_only; };
    std::vector<Cmd> order;
    std::vector<Blk> blocks(1, Blk{false, false, false, false, false, true});
    gcode::Words all= words; // synthesized motion words are appended
    std::vector<uint8_t> block_of(words.size(), 0);
    uint32_t groups= 0;
    for (size_t i= 0; i < words.size(); i++) {
        const gcode::Word &w= words[i];
        Blk *b= &blocks.back();
        if(!is_command(w)) {
            if(is_axis(w.letter)) b->axis= true;
            if(w.letter == 'F') b->feed= true;
            block_of[i]= blocks.size() - 1;
            continue;
        }
        Class c= classify(w);
        bool g53= w.letter == 'G' && w.value == 53;
        if((g53 && b->motion) || (c.group != 0 && (groups & (1u << c.group)))) {
            blocks.push_back(Blk{false, false, false, false, false, true});
            b= &blocks.back();
            groups= 0;
        }
        block_of[i]= blocks.size() - 1;
        if(g53) {
            b->mcs= true;
            continue;
        }
        groups|= 1u << c.group;
        if(c.rank == MOTION) b->motion= true;
        if(c.flags & AXIS_WORDS) b->axis_code= true;
        if(!is_modal_setting(c) || (c.flags & AXIS_WORDS)) b->settings_only= false;
        size_t pos= order.size();
        while(pos > 0 && (order[pos - 1].block > block_of[i] || (order[pos - 1].block == block_of[i] && order[pos - 1].rank > c.rank))) pos--;
        order.insert(order.begin() + pos, Cmd{i, block_of[i], c.rank});
    }

    // a block whose commands are only mode settings moves with the modal motion when it has axis words or G53 (F alone: G1)
    for (size_t k= 0; k < blocks.size(); k++) {
        Blk &b= blocks[k];
        if(b.motion || !(b.mcs || (b.settings_only && (b.axis || b.feed)))) continue;
        all.push_back(gcode::Word{.letter= 'G', .subcode= 0, .has_value= true,
                                  .value= float(b.axis || b.mcs ? modal_group_1 : 1)});
        block_of.push_back(k);
        size_t pos= order.size();
        while(pos > 0 && (order[pos - 1].block > k || (order[pos - 1].block == k && order[pos - 1].rank > MOTION))) pos--;
        order.insert(order.begin() + pos, Cmd{all.size() - 1, uint8_t(k), MOTION});
        b.motion= true;
    }

    for (const Cmd &c : order) {
        const gcode::Word &w= all[c.index];
        Blk &b= blocks[c.block];
        if(c.rank == MOTION && b.mcs && w.value > 1) return fail(stream, "G53 needs G0 or G1");
        if(c.rank == MOTION && b.axis_code) return fail(stream, "G10/G22/G28/G30/G92 cannot share a line with a motion word");
    }
    for (size_t i= 0; i < words.size(); i++) {
        const gcode::Word &w= words[i];
        if(!w.has_value && (blocks[block_of[i]].motion || blocks[block_of[i]].axis_code) && is_move_word(w.letter)) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%c needs a value", w.letter);
            return fail(stream, buf);
        }
    }

    if(order.empty()) order.push_back(Cmd{words.size(), 0, OTHER_M}); // T or S alone

    gcode::Words block_words;
    int current= -1;
    for (size_t n= 0; n < order.size(); n++) {
        const Cmd &c= order[n];
        if(c.block != current) {
            current= c.block;
            block_words.clear();
            for (size_t i= 0; i < all.size(); i++) if(block_of[i] == c.block) block_words.push_back(all[i]);
        }
        size_t index= block_words.size();
        for (size_t i= 0, k= 0; i < all.size(); i++) {
            if(block_of[i] != c.block) continue;
            if(i == c.index) index= k;
            k++;
        }
        Gcode gcode(block_words, index, stream, line);

        if(c.rank == MOTION) {
            gcode.mcs= blocks[c.block].mcs;
            // G80 cancels a canned cycle, so the mode goes back to the last plain motion
            if(!nested && c.index < words.size() && (gcode.g < 4 || (gcode.g >= 81 && gcode.g <= 89))) modal_group_1= gcode.g;
            if(!nested && gcode.g == 80) modal_group_1= 0;
        }

        bool claimed= true;
        if(gcode.has_m) claimed= run_mcode(gcode, nested);
        else if(c.index >= words.size()) run_gcode(gcode, 0, nested);
        else run_gcode(gcode, c.rank == MOTION ? 0 : classify(words[c.index]).flags, nested);

        // a scripted code runs its sub after the modules have seen it, so their handlers still apply;
        // the ok follows when the sub is done, which is the last block of the line by rank
        std::string err;
        if(scripts != nullptr && !nested && scripts->trigger(gcode, stream, err)) {
            return err.empty() || fail(stream, err.c_str());
        }

        // a macro may claim a code no module does, and a nested line never reaches the trigger
        if(!claimed && !nested) {
            char buf[24];
            snprintf(buf, sizeof(buf), "unsupported M%u", gcode.m);
            return fail(stream, buf);
        }

        if(!gcode.error_text.empty()) return fail(stream, gcode.error_text.c_str());
    }
    return true;
}
