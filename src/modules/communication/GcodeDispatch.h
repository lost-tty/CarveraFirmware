/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include <type_traits>
#include "utils/GcodeLine.h"
#include "utils/Parameters.h"

#include <cstdint>
#include <string>

struct SerialMessage;
#include <vector>

class StreamOutput;
class Gcode;

// a module that runs some G/M codes as scripts: return true when the command is taken (err set on failure)
class ScriptHook {
public:
    virtual bool trigger(const Gcode &gcode, StreamOutput *stream, std::string &err) = 0;
};

class GcodeDispatch : public Module
{
public:
    // When an M code runs, relative to the motion around it. Pick IMMEDIATE if the handler does
    // not care where the machine is, ACTION if it must happen at this point in the path, and
    // BARRIER if the next move must not start until it is finished.
    enum When : uint8_t {
        IMMEDIATE,  // runs as the line is read, while queued motion carries on
        ACTION,     // runs at the block it was written before, without stopping; waits like BARRIER until a block can carry one
        BARRIER,    // everything queued before it runs first, and the handler may take as long as it likes
    };

    // One M code an owner claims. The owner keeps the slot, the registry only threads them.
    using McodeFn = void (*)(void *owner, Gcode *);
    struct Mcode {
        uint16_t number;
        When when;
        void *owner;
        McodeFn handler;
        Mcode *next;
    };

    // an owner need not be a Module: a leveling strategy claims its own codes
    static void add_mcode(Mcode &slot, uint16_t number, When when, void *owner, McodeFn handler);

    template<class T, void (T::*M)(Gcode *)> static void add_mcode(Mcode &slot, uint16_t number, When when, T *owner)
    {
        add_mcode(slot, number, when, owner, [](void *self, Gcode *gcode) { (((T *)self)->*M)(gcode); });
    }
#define ADD_MCODE(slot, number, when, method) \
    GcodeDispatch::add_mcode<std::remove_reference<decltype(*this)>::type, &method>(slot, number, GcodeDispatch::when, this)
    static bool run_mcode(Gcode &gcode);
    void report_settings(Gcode *);

    static void add_handler(Module *module); // for a module that handles G or M codes
    void init();

    uint8_t get_modal_command() const { return modal_group_1; }
    bool homed_check_enabled() const { return homed_check; }
    Parameters &parameters() { return params; }
    void set_script_hook(ScriptHook *hook) { scripts= hook; }
    void run_mdi(const SerialMessage &msg); // a console line: refused while a job or script runs
    bool run_line(const SerialMessage &msg); // false: the line was refused
    bool run_line(const std::string &line, StreamOutput *stream);
private:
    enum Gate { PASS, HANDLED, REFUSED };
    bool dispatch(const SerialMessage &msg);
    Gate allowed_while_halted(const gcode::Words &words, StreamOutput *stream);
    Gate homed_enough(const gcode::Words &words, StreamOutput *stream);
    bool execute(const gcode::Words &words, const std::string &text, StreamOutput *stream, unsigned int line);
    bool parameter_statement(const char *p, StreamOutput *stream);
    bool fail(StreamOutput *stream, const char *msg);

    Parameters params;
    Mcode m500, m503;
    static Module *handlers;
    static Mcode *mcodes;
    ScriptHook *scripts= nullptr;
    uint8_t depth= 0; // a line dispatched from inside another must not touch its modal state
    uint8_t modal_group_1;
    bool homed_check;
};
