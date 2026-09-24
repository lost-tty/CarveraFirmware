/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"
#include "libs/Killable.h"
class Gcode;
#include "GcodeFile.h"
#include "Source.h"
#include "libs/McodeRegistry.h"
#include "SimpleShell.h"

#include <stdio.h>
#include <string>
#include <cstdint>
#include <map>
#include <vector>

#include "FreeRTOS.h"

using std::string;

class StreamOutput;

// Job control: plays a file as the bottom source of the stack and feeds the stack from its main loop.
class Player : public Module, public Source, public Killable {

    public:
        void on_module_loaded();
        static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
        bool is_playing() const { return playing_file; }
        bool m1_stops_program() const { return m1_stops; }
        bool get_progress(struct pad_progress &p);
        void on_gcode_received(Gcode *argument);
        void program_stop(Gcode *);
        void optional_stop(Gcode *);
        void suspend_gcode(Gcode *);
        void optional_stop_mode(Gcode *);
        void resume_gcode(Gcode *);

        McodeRegistry::Mcode m0, m1, m333, m334, m600, m601;
        void kill() override {}
        void cleanup() override;
        Source::Result next(SerialMessage &msg) override;
        void abort() override;
        void list(StreamOutput* stream, unsigned around) override;

    private:
        typedef void (Player::*command_t)(string, StreamOutput *);
        static const struct Cmd { const char *name; command_t fn; const char *help; } COMMANDS[];
        SimpleShell::Registered shell_slots[6];
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream );
        void suspend_now();
        void resume_command( string parameters, StreamOutput* stream );
        void goto_command( string parameters, StreamOutput* stream );
        void test_command(string parameters, StreamOutput* stream );

        unsigned long calculate_elapsed_secs();
        unsigned long current_line();
        string extract_options(string& args);
		
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);

        string filename;
        bool verbose;

        GcodeFile file;
        TickType_t start_time;
        struct {
            bool playing_file:1;
            bool m1_stops:1;   // M334 turns it on, M333 off
            bool suspend_pending:1;   // asked for while a script was on top, taken at the next file line
        };
};

extern Player player;
