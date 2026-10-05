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
    {"play",     &Player::play_command,     "play file - play a gcode file"},
    {"progress", &Player::progress_command, "progress [-b] - progress of the file being played"},
    {"abort",    &Player::abort_command,    "abort - stop the machine, held or not, and close the file if one is playing"},
    {"suspend",  &Player::suspend_command,  "suspend [h] - suspend the job, h keeps the spindle on"},
    {"resume",   &Player::resume_command,   "resume - resume a suspended job"},
    {"step",     &Player::step_command,     "step [over|out] - next line, over a sub, out of it"},
    {"goto",     &Player::goto_command,     "goto line - jump to a line while suspended"},
    {"buffer",   &Player::buffer_command,   "buffer <gcode> - queue a gcode line to run before the next file line"},
    {"job",      &Player::job_command,      "job status|watch|load"},
    {nullptr, nullptr, nullptr},
};

void Player::shell(void *self, const char *name, std::string args, StreamOutput *stream)
{
    if(machine_task.is_halted() && strcmp(name, "job") != 0) {
        stream->printf("error:Alarm lock\n");
        return;
    }

    Player *me= static_cast<Player *>(self);
    for (const Cmd *c= COMMANDS; c->name != nullptr; ++c) {
        if(strcmp(c->name, name) == 0) { (me->*(c->fn))(args, stream); return; }
    }
}

void Player::buffer_command( string parameters, StreamOutput *stream )
{
    string err;
    if (!gcode_dispatch.buffer(parameters, stream, err)) {
        stream->printf("error:%s\r\n", err.c_str());
        return;
    }
    stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

bool Player::open_job(const string &path, StreamOutput *stream)
{
    if (program.busy() || program.suspended()) {
        stream->printf("Currently printing, abort print first\r\n");
        return false;
    }

    if (machine_task.is_jogging()) {
        stream->printf("error:busy, jogging\r\n");
        return false;
    }

    string err;
    if (!program.load_job(path, err)) {
        stream->printf("%s\r\n", err.c_str());
        return false;
    }

    THECONVEYOR.clear_executed();
    run_ticks = 0;
    sampled_at = xTaskGetTickCount();
    return true;
}

static bool homed(StreamOutput *stream)
{
    if (machine_task.homed())
        return true;

    stream->printf("error:Machine has not been homed, home first\r\n");
    return false;
}

void Player::play_command( string parameters, StreamOutput *stream )
{
    string name = shift_parameter(parameters);
    if (name.empty() && (program.busy() || program.suspended())) {
        resume_command("", stream);
        return;
    }

    string path = name.empty() ? last.path : absolute_from_relative(name, stream);
    if (path.empty()) {
        stream->printf("error:no job to play\r\n");
        return;
    }

    if (!homed(stream) || !open_job(path, stream))
        return;

    program.resume();
    stream->printf("Playing %s\r\n", path.c_str());
    if (program.job_size() == 0) {
        stream->printf("WARNING - Could not get file size\r\n");
    } else {
        stream->printf("  File size %u\r\n", program.job_size());
    }
}

void Player::job_load( string parameters, StreamOutput *stream )
{
    string path = absolute_from_relative(shift_parameter(parameters), stream);
    if (open_job(path, stream)) {
        stream->printf("Loaded %s\r\n", path.c_str());
    }
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
    watch.tick();
}

const SimpleShell::Sub<Player> Player::JOB_SUBS[] = {
    {"status", &Player::job_status, "job state"},
    {"watch",  &Player::job_watch,  "on|off: push status changes"},
    {"load",   &Player::job_load,   "<file>: load without starting"},
    {nullptr, nullptr, nullptr},
};

void Player::job_command( string parameters, StreamOutput *stream )
{
    SimpleShell::dispatch(this, JOB_SUBS, "job", parameters, stream);
}

const char *Player::phase_name() const
{
    if (program.loaded())
        return "loaded";

    if (program.suspended())
        return "pause";

    return THEKERNEL->get_feed_hold() ? "hold" : "run";
}

static const char *flags()
{
    return program.stepping() ? "single" : "-";
}

// "<role> <phase> <outcome> <flags> <line> <read> <secs> <size> <path>"
static void print_head(StreamOutput *stream, const char *phase, const char *outcome,
                       unsigned long secs, const Program::Head &h)
{
    stream->printf("%s %s %s %s %u %u %lu %lx %s\r\n", h.job ? "file" : "script", phase,
                   outcome, flags(), h.line, h.read, secs, (unsigned long)h.size, h.path);
    string args = program.args(h);
    if (!args.empty()) {
        stream->printf("args %s\r\n", args.c_str());
    }
}

void Player::job_status( string, StreamOutput *stream )
{
    if (!program.busy() && last.path.empty()) {
        stream->printf("none idle - %s 0 0 0 - -\r\n", flags());
        return;
    }
    if (!program.busy()) {
        static const char *const NAMES[] = {"done", "stopped", "halted"};
        print_head(stream, "idle", NAMES[last.how], last.secs, last.head);
        return;
    }
    Program::Head heads[script::Runner::MAX_DEPTH];
    unsigned n = program.heads(heads);
    unsigned long secs = calculate_elapsed_secs();
    for (unsigned i = 0; i < n; i++) {
        print_head(stream, phase_name(), "-", secs, heads[i]);
    }
}

void Player::job_watch( string parameters, StreamOutput *stream )
{
    string what = shift_parameter(parameters);
    if (what == "off") {
        watch.remove(stream);
        stream->printf("job watch off\r\n");
    } else if (what != "on") {
        stream->printf("error:job watch on|off\r\n");
    } else if (watch.add(stream)) {
        stream->printf("job watch on\r\n");
    } else {
        stream->printf("error:%d consoles watch already\r\n", JobWatch::WATCHERS);
    }
}

// the job ended, was stopped or the machine halted
void Player::job_ended(Program::Outcome how)
{
    unsigned long secs = calculate_elapsed_secs();
    unsigned line = program.played_line();
    last = Last{{nullptr, program.job_size(), line, line, true, -1}, program.job_name(), how,
                secs};
    last.head.path = last.path.c_str();
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
    gcode_dispatch.drop_buffered();

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

    p.played_lines = program.played_line();
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

    if(!program.busy()) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    // a tool change or other script is half way in a job; pause at the next file line instead
    if (program.playing() && program.in_sub()) {
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

    if (program.loaded() && !homed(stream))
        return;

    program.resume();
    stream->printf("Playing file resumed\n");
}

void Player::step_command(string parameters, StreamOutput *stream)
{
    string how = shift_parameter(parameters);
    if (how != "" && how != "over" && how != "out") {
        stream->printf("error:step [over|out]\r\n");
        return;
    }

    // a feed hold on a job or a script counts as suspended
    if (!program.suspended() && !(program.busy() && THEKERNEL->get_feed_hold())) {
        stream->printf("Not suspended\n");
        return;
    }

    if (program.loaded() && !homed(stream))
        return;

    if (!machine_task.standing()) {
        stream->printf("error:still moving, step when it stands\r\n");
        return;
    }

    program.step(how == "over" ? Program::OVER : how == "out" ? Program::OUT : Program::INTO);
}