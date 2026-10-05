/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"
class Gcode;
#include "libs/McodeRegistry.h"
#include "SimpleShell.h"
#include "JobWatch.h"
#include "Program.h"

#include <stdio.h>
#include <string>
#include <cstdint>
#include <map>
#include <vector>

#include "FreeRTOS.h"

using std::string;

class StreamOutput;

// Job control: the shell commands and M-codes for the job the program module runs.
class Player : public Module {

    public:
        void on_module_loaded();
        void service();
        static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
        bool m1_stops_program() const { return m1_stops; }
        bool get_progress(struct pad_progress &p);
        void on_gcode_received(Gcode *argument);
        void program_stop(Gcode *);
        void optional_stop(Gcode *);
        void suspend_gcode(Gcode *);
        void optional_stop_mode(Gcode *);
        void resume_gcode(Gcode *);
        void progress_report(Gcode *);

        McodeRegistry::Mcode m0, m1, m27, m333, m334, m600, m601;
        void job_ended(Program::Outcome how);
        void job_status(string parameters, StreamOutput *stream);
        void unwatch(StreamOutput *stream) { watch.remove(stream); }
        void suspend(StreamOutput *stream);
        void resume(StreamOutput *stream);

    private:
        typedef void (Player::*command_t)(string, StreamOutput *);
        static const struct Cmd { const char *name; command_t fn; const char *help; } COMMANDS[];
        SimpleShell::Registered shell_slots[10];
        static const SimpleShell::Sub<Player> JOB_SUBS[];
        void play_command( string parameters, StreamOutput* stream );
        bool open_job(const string &path, StreamOutput *stream);
        void job_load( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream );
        void suspend_now();
        void resume_command( string parameters, StreamOutput* stream );
        void step_command( string parameters, StreamOutput* stream );
        void goto_command( string parameters, StreamOutput* stream );
        void buffer_command( string parameters, StreamOutput* stream );
        void job_command( string parameters, StreamOutput* stream );
        void job_watch( string parameters, StreamOutput* stream );
        const char *phase_name() const;
        void test_command(string parameters, StreamOutput* stream );

        unsigned long calculate_elapsed_secs();
        void sample_runtime();
		
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);

        TickType_t run_ticks = 0, sampled_at = 0;
        bool m1_stops = false;   // M334 turns it on, M333 off
        JobWatch watch{*this};
        // owns the path; the job's goes when it ends
        struct Last {
            Program::Head head;
            string path;
            Program::Outcome how;
            unsigned long secs;
        };
        Last last{{nullptr, 0, 0, 0, true, -1}, "", Program::DONE, 0};
};

extern Player player;
