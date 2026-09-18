/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"
class Gcode;
#include "GcodeFile.h"
#include "Source.h"
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
class Player : public Module, public Source {

    public:
        void on_module_loaded();
        static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
        void on_main_loop( void* argument );
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(Gcode *argument);
        void on_halt(void *argument);
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
        void suspend_now( StreamOutput* stream );
        void resume_command( string parameters, StreamOutput* stream );
        void goto_command( string parameters, StreamOutput* stream );
        void test_command(string parameters, StreamOutput* stream );

        unsigned long calculate_elapsed_secs();
        unsigned long current_line();
        string extract_options(string& args);
		
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);

        string filename;
        string last_filename;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;


        GcodeFile file;
        TickType_t start_time;
        unsigned long goto_line;
        unsigned int playing_lines;
        uint8_t current_motion_mode;
        float saved_position[3]; // only saves XYZ
        float slope;
        std::map<uint16_t, float> saved_temperatures;
        struct {
            bool booted:1;
            bool home_on_boot:1;
            bool playing_file:1;
            bool leave_heaters_on:1;
            bool override_leave_heaters_on:1;
            bool suspend_pending:1;   // asked for while a script was on top, taken at the next file line
            bool laser_clustering:1;
        };
};
