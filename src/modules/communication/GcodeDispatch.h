/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/McodeRegistry.h"
#include "utils/GcodeLine.h"
#include "utils/Parameters.h"
#include "modules/robot/MachineTask.h"

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
    static bool run_mcode(Gcode &gcode, bool nested);
    void report_settings(Gcode *);

    static void add_handler(Module *module); // for a module that handles G or M codes
    void init();

    uint8_t get_modal_command() const { return modal_group_1; }
    bool homed_check_enabled() const { return homed_check; }
    Parameters &parameters() { return params; }
    void set_script_hook(ScriptHook *hook) { scripts= hook; }
    void run_mdi(const SerialMessage &msg); // a console line: most of them wait for the job to finish
    // nested: dispatched by a handler, so it must not touch the enclosing line's modal state
    bool run_line(const SerialMessage &msg, bool nested= false); // false: the line was refused
    bool run_line(const std::string &line, StreamOutput *stream, bool nested= true);
private:
    enum Gate { PASS, HANDLED, REFUSED };
    bool dispatch(const SerialMessage &msg, bool nested);
    Gate allowed_while_halted(const gcode::Words &words, StreamOutput *stream);
    Gate homed_enough(const gcode::Words &words, StreamOutput *stream);
    bool execute(const gcode::Words &words, const std::string &text, StreamOutput *stream, unsigned int line, bool nested);
    bool parameter_statement(const char *p, StreamOutput *stream);
    bool fail(StreamOutput *stream, const char *msg);
    static bool safe_while_running(const gcode::Words &words);
    static void broadcast(Gcode &gcode, OnMachine);
    static void broadcast_drained(Gcode &gcode, OnMachine);
    static void run_barrier(Gcode &gcode, OnMachine);
    static void hold_or_run(Gcode &gcode, OnMachine);
    void run_gcode(Gcode &gcode, uint8_t flags, bool nested);

    Parameters params;
    McodeRegistry::Mcode m500, m503;
    static Module *handlers;
    ScriptHook *scripts= nullptr;
    uint8_t modal_group_1;
    bool homed_check;
};
