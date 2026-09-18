/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
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
    static void add_handler(Module *module); // for a module that handles G or M codes
    void init();

    uint8_t get_modal_command() const { return modal_group_1; }
    Parameters &parameters() { return params; }
    void set_script_hook(ScriptHook *hook) { scripts= hook; }
    void run_mdi(const SerialMessage &msg); // a console line: refused while a job or script runs
    void run_line(const SerialMessage &msg);
    void run_line(const std::string &line, StreamOutput *stream);
private:
    void dispatch(const SerialMessage &msg);
    bool allowed_while_halted(const gcode::Words &words, StreamOutput *stream);
    bool homed_enough(const gcode::Words &words, StreamOutput *stream);
    void execute(const gcode::Words &words, const std::string &text, StreamOutput *stream, unsigned int line);
    void parameter_statement(const char *p, StreamOutput *stream);
    void fail(StreamOutput *stream, const char *msg);
    void halt();

    Parameters params;
    static Module *handlers;
    ScriptHook *scripts= nullptr;
    uint8_t depth= 0; // a line dispatched from inside another must not touch its modal state
    uint8_t modal_group_1;
    bool homed_check;
};
