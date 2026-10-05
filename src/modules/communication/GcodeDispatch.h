/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/Killable.h"
#include "libs/McodeRegistry.h"
#include "utils/GcodeLine.h"
#include "utils/Parameters.h"
#include "modules/robot/MachineTask.h"

#include <cstdint>
#include <deque>
#include <string>

struct SerialMessage;
#include <vector>

class StreamOutput;
class Gcode;

// a module that runs some G/M codes as scripts: return true when the command is taken (err set on failure)
class ScriptHook {
public:
    virtual bool trigger(const Gcode &gcode, StreamOutput *stream, std::string &err) = 0;
    virtual bool call(const std::string &line, StreamOutput *stream, std::string &err) = 0;
};

class GcodeDispatch : public Module, public Killable
{
public:
    void service();
    void kill() override {}
    void cleanup() override { buffered.clear(); }
    static bool run_mcode(Gcode &gcode);
    void report_settings(Gcode *);
    void say(Gcode *);   // M118

    static void add_handler(Module *module); // for a module that handles G or M codes
    void init();

    uint8_t get_modal_command() const { return modal_cycle != 0 ? modal_cycle : modal_motion; }
    bool set_modal_command(unsigned g);   // G0..G3 or G81..G89, without moving
    float get_cycle_initial() const { return cycle_initial; }

    bool homed_check_enabled() const { return homed_check; }
    Parameters &parameters() { return params; }
    void set_script_hook(ScriptHook *hook) { scripts= hook; }
    void run_mdi(const SerialMessage &msg);
    // queued console G-code, also taken while a job plays
    bool buffer(const std::string &line, StreamOutput *stream, std::string &err);
    void drop_buffered() { buffered.clear(); }
    void program_end();
private:
    enum Gate { PASS, HANDLED, REFUSED };
    bool run_line(const SerialMessage &msg);
    Gate allowed_while_halted(const gcode::Words &words);
    Gate homed_enough(const gcode::Words &words);
    bool execute(const gcode::Words &words, const std::string &text, unsigned int line);
    bool parameter_statement(const char *p);
    bool announce(const std::string &line, size_t from, unsigned int number,
                  const gcode::ParamStore *store);
    bool fail(const char *msg);
    static bool safe_while_running(const gcode::Words &words);
    static void broadcast(Gcode &gcode, OnMachine);
    static void broadcast_drained(Gcode &gcode, OnMachine);
    static void run_barrier(Gcode &gcode, OnMachine);
    static void hold_or_run(Gcode &gcode, OnMachine);
    void run_gcode(Gcode &gcode, uint8_t flags);

    Parameters params;
    McodeRegistry::Mcode m500, m503, m118;
    static Module *handlers;
    ScriptHook *scripts= nullptr;
    uint8_t modal_motion;
    uint8_t modal_cycle;
    float cycle_initial;
    bool homed_check;
    struct Buffered {
        std::string line;
        StreamOutput *stream;
    };
    std::deque<Buffered> buffered;
    static const unsigned BUFFER_LIMIT = 32; // a remote client must not grow it without bound
};
