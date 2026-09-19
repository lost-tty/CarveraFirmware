/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "GcodeDispatch.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "Conveyor.h"
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
                      TOOL_OFFSET, WCS, PATH, DISTANCE, RETRACT, NON_MODAL, MOTION, OTHER_M, STOP };

enum Flag : uint8_t {
    AXIS_WORDS = 1,   // takes the axis words itself, so it cannot share a block with a motion word
    NEEDS_HOMED = 2,  // probing, tool change and the ATC moves: refused on an unhomed machine
    WHEN_HALTED = 4,  // still runs while halted: status queries and things that turn stuff off
};

struct Class { uint8_t group; Rank rank; uint8_t flags; };  // group 0: may be combined freely

static bool is_command(const gcode::Word &w) { return w.letter == 'G' || w.letter == 'M'; }

static Class classify(const gcode::Word &w)
{
    unsigned n= w.value;
    if(w.letter == 'G') {
        if(n <= 3 || n == 73 || (n >= 80 && n <= 89)) return {1, MOTION, 0};
        if(n == 38) return {1, MOTION, NEEDS_HOMED};
        switch(n) {
            case 4: return {0, DWELL, 0};
            case 10: return {0, NON_MODAL, AXIS_WORDS};
            case 17: case 18: case 19: return {2, PLANE, 0};
            case 20: case 21: return {6, UNITS, 0};
            case 28: return {0, NON_MODAL, AXIS_WORDS};
            case 30: return {0, NON_MODAL, uint8_t(AXIS_WORDS | NEEDS_HOMED)};
            case 31: case 32: return {0, NON_MODAL, AXIS_WORDS};
            case 40: case 41: case 42: return {7, CUTTER_COMP, 0};
            case 43: case 49: return {8, TOOL_OFFSET, 0};
            case 54: case 55: case 56: case 57: case 58: case 59: return {12, WCS, 0};
            case 61: case 64: return {13, PATH, 0};
            case 90: case 91: return {uint8_t(w.subcode == 0 ? 3 : 4), DISTANCE, 0};
            case 92: return {0, NON_MODAL, uint8_t(w.subcode == 0 ? AXIS_WORDS : 0)};
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
}

void GcodeDispatch::halt()
{
    THEKERNEL->set_halt_reason(MANUAL);
    THEKERNEL->call_event(ON_HALT, nullptr);
}

// nothing of the line has run yet: the reply is enough unless a job or script would go on past it
void GcodeDispatch::fail(StreamOutput *stream, const char *msg)
{
    stream->printf("error:%s\r\n", msg);
    if(!sources.empty() || !THECONVEYOR.is_idle()) halt();
}


// an interleaved line would move the machine out of sequence; a suspended job is safe to jog
void GcodeDispatch::run_mdi(const SerialMessage &msg)
{
    if(sources.active()) {
        msg.stream->printf("error:busy, a job or script is running\r\n");
        return;
    }
    run_line(msg);
}

void GcodeDispatch::run_line(const SerialMessage &msg)
{
    depth++;
    dispatch(msg);
    depth--;
}

void GcodeDispatch::run_line(const std::string &line, StreamOutput *stream)
{
    run_line(SerialMessage{stream, line, 0});
}

void GcodeDispatch::dispatch(const SerialMessage &msg)
{
    const string &s= msg.message;

    size_t i= s.find_first_not_of(" \t");
    if(i == string::npos) {
        msg.stream->printf("ok\r\n");
        return;
    }

    char c= s[i];
    if(c == '$' || islower((unsigned char)c)) return; // simpleshell command

    size_t j= i;
    if(c == 'N') {
        j++;
        while(j < s.size() && isdigit(s[j])) j++;
        while(j < s.size() && s[j] == ' ') j++;
    }
    if(j < s.size() && s[j] == '#') {
        parameter_statement(s.c_str() + j, msg.stream);
        return;
    }

    gcode::Line parsed; // local: modules may dispatch console lines while a line executes
    if(!parsed.parse(s.c_str() + i, &params)) {
        fail(msg.stream, parsed.error_text().c_str());
        return;
    }
    execute(parsed.words(), s.substr(i), msg.stream, msg.line);
}

// "#n = expr" assigns, "#n" prints
void GcodeDispatch::parameter_statement(const char *p, StreamOutput *stream)
{
    std::string err;
    if(strchr(p, '=') != nullptr) {
        if(!gcode::assign(p, params, err)) {
            fail(stream, err.c_str());
            return;
        }
    } else {
        char *end;
        int n= strtol(p + 1, &end, 10);
        float v;
        if(end == p + 1) {
            fail(stream, "bad parameter number");
            return;
        }
        if(params.get(n, v)) stream->printf("#%d = %.4f\r\n", n, v);
        else stream->printf("#%d not set\r\n", n);
    }
    stream->printf("ok\r\n");
}

// M999 is the only way out of a halt, so it is handled before the alarm lock refuses everything else
bool GcodeDispatch::allowed_while_halted(const gcode::Words &words, StreamOutput *stream)
{
    if(!THEKERNEL->is_halted()) return true;

    for (const gcode::Word &w : words) {
        if(w.letter == 'M' && w.value == 999) {
            THEKERNEL->clear_halt();
            stream->printf("WARNING: After HALT you should HOME as position is currently unknown\nok\n");
            return false;
        }
    }
    for (const gcode::Word &w : words) {
        if(!is_command(w)) continue;
        if(classify(w).flags & WHEN_HALTED) continue;
        stream->printf("error:Alarm lock\n");
        return false;
    }
    return true;
}

bool GcodeDispatch::homed_enough(const gcode::Words &words, StreamOutput *stream)
{
    for (const gcode::Word &w : words) {
        if(w.letter == 'M' && (w.value == 887 || w.value == 888)) {
            homed_check= (w.value == 887);
            stream->printf("Homed check %s\nok\n", homed_check ? "enabled" : "disabled");
            return false;
        }
        if(!homed_check || !is_command(w)) continue;
        if((classify(w).flags & NEEDS_HOMED) && !THEROBOT.is_homed_all_axes()) {
            stream->printf("error:Machine has not been homed, home first (M888 disables this check)\n");
            THEKERNEL->set_halt_reason(NON_HOME);
            THEKERNEL->call_event(ON_HALT, nullptr);
            return false;
        }
    }
    return true;
}

void GcodeDispatch::execute(const gcode::Words &words, const string &text, StreamOutput *stream, unsigned int line)
{
    if(words.empty()) {
        stream->printf("ok\r\n");
        return;
    }

    if(!allowed_while_halted(words, stream)) return;
    if(!homed_enough(words, stream)) return;

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
            if(strchr("XYZABC", w.letter)) b->axis= true;
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
        if(!is_modal_setting(c)) b->settings_only= false;
        size_t pos= order.size();
        while(pos > 0 && (order[pos - 1].block > block_of[i] || (order[pos - 1].block == block_of[i] && order[pos - 1].rank > c.rank))) pos--;
        order.insert(order.begin() + pos, Cmd{i, block_of[i], c.rank});
    }

    // a block whose commands are only mode settings moves with the modal motion when it has axis words or G53 (F alone: G1)
    for (size_t k= 0; k < blocks.size(); k++) {
        Blk &b= blocks[k];
        if(b.motion || !(b.mcs || (b.settings_only && (b.axis || b.feed)))) continue;
        all.push_back(gcode::Word{'G', 0, float(b.axis || b.mcs ? modal_group_1 : 1), true});
        block_of.push_back(k);
        size_t pos= order.size();
        while(pos > 0 && (order[pos - 1].block > k || (order[pos - 1].block == k && order[pos - 1].rank > MOTION))) pos--;
        order.insert(order.begin() + pos, Cmd{all.size() - 1, uint8_t(k), MOTION});
        b.motion= true;
    }

    for (const Cmd &c : order) {
        const gcode::Word &w= all[c.index];
        Blk &b= blocks[c.block];
        if(c.rank == MOTION && b.mcs && w.value > 1) {
            fail(stream, "G53 needs G0 or G1");
            return;
        }
        if(c.rank == MOTION && b.axis_code) {
            fail(stream, "G10/G28/G30/G92 cannot share a line with a motion word");
            return;
        }
    }
    for (size_t i= 0; i < words.size(); i++) {
        const gcode::Word &w= words[i];
        if(!w.has_value && blocks[block_of[i]].motion && strchr("XYZABCIJKRF", w.letter)) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%c needs a value", w.letter);
            fail(stream, buf);
            return;
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
        Gcode gcode(block_words, index, text, stream, line);

        if(c.rank == MOTION) {
            gcode.mcs= blocks[c.block].mcs;
            // G80 cancels a canned cycle, so the mode goes back to the last plain motion
            if(depth == 1 && c.index < words.size() && (gcode.g < 4 || (gcode.g >= 81 && gcode.g <= 89))) modal_group_1= gcode.g;
            if(depth == 1 && gcode.g == 80) modal_group_1= 0;
        }

        for (Module *m = handlers; m != nullptr; m = m->next_gcode_handler) m->on_gcode_received(&gcode);

        // a scripted code runs its sub after the modules have seen it, so their handlers still apply;
        // the ok follows when the sub is done, which is the last block of the line by rank
        std::string err;
        if(scripts != nullptr && depth == 1 && scripts->trigger(gcode, stream, err)) {
            if(!err.empty()) fail(stream, err.c_str());
            return;
        }

        if(gcode.is_error) {
            stream->printf("error:%s\r\n", gcode.txt_after_ok.empty() ? "unknown" : gcode.txt_after_ok.c_str());
            halt();
            return;
        }
        if(gcode.add_nl) stream->printf("\r\n");
        if(!gcode.txt_after_ok.empty()) {
            stream->printf("ok %s\r\n", gcode.txt_after_ok.c_str());
        } else if(!THEKERNEL->is_ok_per_line() || n + 1 == order.size()) {
            stream->printf("ok\r\n");
        }
    }
}
