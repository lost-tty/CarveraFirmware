/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "SimpleShell.h"
#include "Robot.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/Logging.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "GcodeDispatch.h"
#include "modules/tools/ToolHead.h"
#include "checksumm.h"
#include "SDFAT.h"

#include "modules/robot/Conveyor.h"
#include "modules/robot/MachineTask.h"
#include "DirHandle.h"
#include "PlayerPublicAccess.h"
#include "Program.h"
#include "TemperatureControlPool.h"
#include "Block.h"

#include <math.h>

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

extern SDFAT mounter;

void Player::on_module_loaded()
{
    for (unsigned i= 0; COMMANDS[i].name != nullptr; i++) SimpleShell::add_command(shell_slots[i], COMMANDS[i].name, &Player::shell, this, COMMANDS[i].help);
    GcodeDispatch::add_handler(this);
    register_for_event(ON_MAIN_LOOP);
    ADD_MCODE(m0, 0, BARRIER, Player::program_stop);
    ADD_MCODE(m27, 27, BESIDE_JOB, Player::progress_report);
    ADD_MCODE(m333, 333, IMMEDIATE, Player::optional_stop_mode);
    ADD_MCODE(m334, 334, IMMEDIATE, Player::optional_stop_mode);
    ADD_MCODE(m1, 1, BARRIER, Player::optional_stop);
    ADD_MCODE(m600, 600, BARRIER, Player::suspend_gcode);
    ADD_MCODE(m601, 601, IMMEDIATE, Player::resume_gcode);
}

unsigned long Player::calculate_elapsed_secs()
{
    sample_runtime();
    return (run_ticks + configTICK_RATE_HZ / 2) / configTICK_RATE_HZ;
}

void Player::sample_runtime()
{
    TickType_t now = xTaskGetTickCount();
    if (program.playing() && !program.suspended() && !THEKERNEL->get_feed_hold()
        && !machine_task.is_halted()) {
        run_ticks += now - sampled_at;
    }
    sampled_at = now;
}


// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

// only G codes come through here; the M codes are registered
void Player::on_gcode_received(Gcode *argument)
{
    Gcode *gcode = argument;
    if(!gcode->has_g() || gcode->g() != 28) return;

    // homing cancels suspend
    if (program.suspended()) program.resume();
}

// M0: stop the program until the operator resumes it
// a barrier runs on the machine task, which cannot wait for itself to drain: the main loop
// picks this up and suspends there
void Player::program_stop(Gcode *)
{
    program.ask_pause();
}

// M1: the same, obeyed only when the optional stop mode is on
void Player::optional_stop(Gcode *)
{
    if(!m1_stops) return;
    program.ask_pause();
}

// M333, M334: whether M1 stops the program
void Player::optional_stop_mode(Gcode *gcode)
{
    m1_stops= gcode->m() == 334;
    printk("turning optional stop mode %s\r\n", m1_stops ? "on" : "off");
}

// M600: suspend
void Player::suspend_gcode(Gcode *)
{
    program.ask_pause();
}

void Player::progress_report(Gcode *)
{
    progress_command("-b", &THEKERNEL->streams);
}

void Player::resume_gcode(Gcode *gcode)
{
    this->resume_command("", &THEKERNEL->streams);
}

// When a new line is received, check if it is a command, and if it is, act upon it
const Player::Cmd Player::COMMANDS[] = {
    {"play",     &Player::play_command,     "play file [-v] - play a gcode file"},
    {"progress", &Player::progress_command, "progress [-b] - progress of the file being played"},
    {"abort",    &Player::abort_command,    "abort - stop the machine, held or not, and close the file if one is playing"},
    {"suspend",  &Player::suspend_command,  "suspend [h] - suspend the job, h keeps the spindle on"},
    {"resume",   &Player::resume_command,   "resume - resume a suspended job"},
    {"goto",     &Player::goto_command,     "goto line - jump to a line while suspended"},
    {"buffer",   &Player::buffer_command,   "buffer <gcode> - queue a gcode line to run before the next file line"},
    {nullptr, nullptr, nullptr},
};

void Player::shell(void *self, const char *name, std::string args, StreamOutput *stream)
{
    if(machine_task.is_halted()) return;
    Player *me= static_cast<Player *>(self);
    for (const Cmd *c= COMMANDS; c->name != nullptr; ++c) {
        if(strcmp(c->name, name) == 0) { (me->*(c->fn))(args, stream); return; }
    }
}

void Player::buffer_command( string parameters, StreamOutput *stream )
{
    string err;
    if (!gcode_dispatch.offer(parameters, stream, true, err)) {
        stream->printf("error:%s\r\n", err.c_str());
        return;
    }
    stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{

    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    string path = absolute_from_relative(shift_parameter(parameters), stream);

    if (program.busy() || program.suspended()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    if (!machine_task.homed()) {
        stream->printf("error:Machine has not been homed, home first\r\n");
        return;
    }

    // -v echoes every line, to everyone: the stream that asked may be gone by then
    bool verbose = options.find_first_of("Vv") != string::npos;
    string err;
    if (!program.start_job(path, verbose, err)) {
        stream->printf("%s\r\n", err.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", path.c_str());
    THECONVEYOR.clear_executed();

    if (program.job_size() == 0) {
        stream->printf("WARNING - Could not get file size\r\n");
    } else {
        stream->printf("  File size %u\r\n", program.job_size());
    }
    run_ticks = 0;
    sampled_at = xTaskGetTickCount();
}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{
    if (!program.suspended()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

    if (!program.playing()) {
    	stream->printf("Missing file handle!\r\n");
    	return;
    }

    string line_str = shift_parameter(parameters);
    if (!line_str.empty()) {
        char *ptr = NULL;
        unsigned long line = strtol(line_str.c_str(), &ptr, 10);
        if(line < 1) line = 1;
        string err;
        if (!program.jump(line, err)) {
            stream->printf("error:%s\r\n", err.c_str());
            return;
        }
        stream->printf("Goto line %lu...\r\n", line);
        machine_task.post_stop();
    }
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!program.playing()) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    unsigned size = program.job_size(), read = program.job_read();
    if(size > 0) {
        unsigned long est = 0;
        unsigned long elapsed_secs = calculate_elapsed_secs();
        if(elapsed_secs > 10) {
            unsigned long bytespersec = read / elapsed_secs;
            if(bytespersec > 0)
                est = (size - read) / bytespersec;
        }

        float pcnt = read * 100.0F / size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf(est > 0 ? "file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu, est time: %02lu:%02lu:%02lu\r\n"
                                   : "file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu\r\n",
                           program.job_name(), (unsigned int)roundf(pcnt),
                           elapsed_secs / 3600, (elapsed_secs % 3600) / 60, elapsed_secs % 60,
                           est / 3600, (est % 3600) / 60, est % 60);
        } else {
            stream->printf("SD printing byte %u/%u\r\n", read, size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::on_main_loop(void *)
{
    sample_runtime();
}

// the job ended, was stopped or the machine halted
void Player::job_ended()
{
    unsigned long secs = calculate_elapsed_secs();
    printk("%s ran for %02lu:%02lu:%02lu\n", program.job_name(),
           secs / 3600, (secs % 3600) / 60, secs % 60);
    m1_stops = false;
}

// stops whatever the machine is doing, held or not, and closes the file if one is open: a hold
// in MDI has no other way out than this
void Player::abort_command( string parameters, StreamOutput *stream )
{
    bool file= program.busy();
    program.stop(); // the file and any script on top of it, or a script alone
    gcode_dispatch.drop_offered();

    if(machine_task.is_halted()) {
        printk("Aborted by halt\n");
        return;
    }

    if (parameters.empty()) {
        if(machine_task.post_stop()) tool_head.stop_all();
        stream->printf(file ? "Aborted playing or paused file. \r\n" : "Stopped\r\n");
    }
}


/*
bool Player::check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value)
{
	float new_slope = 0.0;
	bool is_cluster = false;
	Gcode *gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
	if (!gcode->has_m() && gcode->has_g() && gcode->g() == 1) {
		*x_value = gcode->get_value('X');
		*y_value = gcode->get_value('Y');
		*s_value = gcode->get_value('S');
		*distance = sqrtf((*x_value) * (*x_value) + (*y_value) * (*y_value));
		if (*x_value == 0) {
			new_slope = *y_value > 0 ? 1000 : -1000;
		} else if (*y_value == 0) {
			new_slope = *x_value > 0 ? 0.001 : -0.001;
		} else {
			new_slope = *y_value / *x_value;
		}
		if ((*distance) < 1.0 && fabs (new_slope - *slope) < 0.1) {
			is_cluster = true;
		}
		*slope = new_slope;
	}
	delete gcode;

	return is_cluster;
}
*/

bool Player::get_progress(struct pad_progress &p)
{
    if(program.job_size() == 0)
        return false;

    p.played_lines = machine_task.where().line;
    p.elapsed_secs = this->calculate_elapsed_secs();
    p.percent_complete = roundf(program.job_read() * 100.0F / program.job_size());
    p.filename = program.job_name();
    return true;
}

void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if (program.suspended()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!program.playing()) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    // a tool change or other script is half way; pause at the next file line instead
    if (program.in_sub()) {
        program.ask_pause();
        stream->printf("Suspending after the running script...\n");
        return;
    }
    suspend_now();
}

void Player::suspend_now()
{
    program.suspend();
    printk("Suspended, resume to continue playing\n");
}

void Player::resume_command(string parameters, StreamOutput *stream )
{
    if (program.cancel_pause()) {
        stream->printf("Suspend cancelled\n");
        return;
    }
    if(!program.suspended()) {
        stream->printf("Not suspended\n");
        return;
    }

    program.resume();
    stream->printf("Playing file resumed\n");
}