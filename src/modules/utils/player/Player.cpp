/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "SimpleShell.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/Logging.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "GcodeDispatch.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "SDFAT.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "ATCHandlerPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "ScriptsPublicAccess.h"
#include "Scripts.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "Block.h"

#include <math.h>

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")
#define laser_module_clustering_checksum 	  CHECKSUM("laser_module_clustering")

extern SDFAT mounter;

// runs a machine script sub if it exists; false when nothing ran
static bool run_script(const char *sub, const float *args, unsigned nargs)
{
    return scripts.run_sub(sub, args, nargs);
}

void Player::on_module_loaded()
{
    this->playing_file = false;
    this->start_time = xTaskGetTickCount();
    this->reply_stream = nullptr;
    this->suspend_pending = false;
    this->slope = 0.0;

    for (unsigned i= 0; COMMANDS[i].name != nullptr; i++) SimpleShell::add_command(shell_slots[i], COMMANDS[i].name, &Player::shell, this, COMMANDS[i].help);
    GcodeDispatch::add_handler(this);

    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();

    this->laser_clustering = THEKERNEL->config->value(laser_module_clustering_checksum)->by_default(false)->as_bool();
}

unsigned long Player::calculate_elapsed_secs()
{
    TickType_t now = xTaskGetTickCount();

    // Handle tick count overflow
    TickType_t elapsedTicks = (now >= start_time) ? (now - start_time) : (now + (portMAX_DELAY - start_time + 1));

    return (pdTICKS_TO_MS(elapsedTicks) + 500) / 1000;
}

void Player::cleanup()
{
    if(THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        THEKERNEL->set_waiting(false);
        sources.resume();
        THEROBOT.pop_state();
        printk("Suspend cleared\n");
    }
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

void Player::on_gcode_received(Gcode *argument)
{
    Gcode *gcode = argument;
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 1) { //optiional stop
            if (THEKERNEL->get_optional_stop_mode()){
                this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);
            }
        } else if (gcode->m == 600) { // suspend, M600.1 leaves the spindle on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);
        } else if (gcode->m == 601) { // resume
            this->resume_command("", gcode->stream);
        }
    } else if(gcode->has_g) {
        if (gcode->g == 28) { // homing cancels suspend
            if (THEKERNEL->is_suspending()) {
                sources.resume();
                THEROBOT.pop_state();
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
const Player::Cmd Player::COMMANDS[] = {
    {"play",     &Player::play_command,     "play file [-v] - play a gcode file"},
    {"progress", &Player::progress_command, "progress [-b] - progress of the file being played"},
    {"abort",    &Player::abort_command,    "abort - abort the file being played"},
    {"suspend",  &Player::suspend_command,  "suspend [h] - suspend the job, h keeps the spindle on"},
    {"resume",   &Player::resume_command,   "resume - resume a suspended job"},
    {"goto",     &Player::goto_command,     "goto line - jump to a line while suspended"},
    {nullptr, nullptr, nullptr},
};

void Player::shell(void *self, const char *name, std::string args, StreamOutput *stream)
{
    if(THEKERNEL->is_halted()) return;
    Player *me= static_cast<Player *>(self);
    for (const Cmd *c= COMMANDS; c->name != nullptr; ++c) {
        if(strcmp(c->name, name) == 0) { (me->*(c->fn))(args, stream); return; }
    }
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{

    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(shift_parameter(parameters));
    this->last_filename = this->filename;

    if (!sources.empty() || THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    if (!THEROBOT.is_homed_all_axes()) {
        stream->printf("error:Machine has not been homed, home first\r\n");
        THEKERNEL->halt(NON_HOME, "machine is not homed");
        return;
    }

    if (!file.open(this->filename.c_str())) { // also closes a paused print
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", this->filename.c_str());

    this->playing_file = true;
    sources.push(this);

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        // we send to the kernels stream as it cannot go away
        this->current_stream = &THEKERNEL->streams;
    }

    if (file.size() == 0) {
        stream->printf("WARNING - Could not get file size\r\n");
    } else {
        stream->printf("  File size %ld\r\n", file.size());
    }
    this->start_time = xTaskGetTickCount();
    this->playing_lines = 0;
    this->goto_line = 0;

    // force into absolute mode
    THEROBOT.set_absolute_mode();

    // reset current position;
    THEROBOT.reset_position_from_current_actuator_position();
}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{
    if (!THEKERNEL->is_suspending()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

    if (!file.is_open()) {
    	stream->printf("Missing file handle!\r\n");
    	return;
    }

    string line_str = shift_parameter(parameters);
    if (!line_str.empty()) {
        char *ptr = NULL;
        this->goto_line = strtol(line_str.c_str(), &ptr, 10);
        this->goto_line = this->goto_line < 1 ? 1 : this->goto_line;
        stream->printf("Goto line %lu...\r\n", this->goto_line);
        file.seek_line(this->goto_line);
    }
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && file.is_open()) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", file.bytes(), file.size());
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", file.bytes(), file.size());
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file.size() > 0) {
        unsigned long est = 0;
        unsigned long elapsed_secs = calculate_elapsed_secs();
        if(elapsed_secs > 10) {
            unsigned long bytespersec = file.bytes() / elapsed_secs;
            if(bytespersec > 0)
                est = (file.size() - file.bytes()) / bytespersec;
        }

        float pcnt = file.bytes() * 100.0F / file.size();
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), elapsed_secs / 3600, (elapsed_secs % 3600) / 60, elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", file.bytes(), file.size());
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

// the motion line executing, or the last line fed when none is or a script is on top
unsigned long Player::current_line()
{
    if (sources.top() == this) {
        // the is_ready flag is cleared first when the ISR drops a block, so a ready block stays valid while we read it
        const Block *block = THEKERNEL->step_ticker.get_current_block();
        if (block != nullptr && block->is_ready && block->is_g123) return block->line;
    }
    return file.lines();
}

void Player::list(StreamOutput *stream, unsigned around)
{
    stream->printf("%s:\r\n", this->filename.c_str());
    file.list(stream, current_line(), around);
}

// the file ended, was aborted or the machine halted
void Player::abort()
{
    this->playing_file = false;
    this->suspend_pending = false;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->filename = "";
    this->current_stream = NULL;
    file.close();
}

void Player::abort_command( string parameters, StreamOutput *stream )
{
    if(sources.empty()) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    sources.clear(); // the file and any script on top of it, or a script alone
    sources.resume();
    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THECONVEYOR.wait_for_idle();

    if(THEKERNEL->is_halted()) {
        printk("Aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);

    // turn off spindle
    {
		gcode_dispatch.run_line("M5", &StreamOutput::NullStream);
    }

    if (parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THECONVEYOR.flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT.reset_position_from_current_actuator_position();
        stream->printf("Aborted playing or paused file. \r\n");
    }
}


Source::Result Player::next(SerialMessage &msg)
{
    if (this->suspend_pending) {
        this->suspend_pending = false;
        suspend_now(&THEKERNEL->streams);
        return WAIT;
    }

    char buf[130];
    unsigned long discarded = file.discarded();
    if (file.next_line(buf, sizeof(buf))) {
        if (this->current_stream != nullptr) {
            if (file.discarded() != discarded) this->current_stream->printf("Warning: Discarded long line\n");
            this->current_stream->printf("%s", buf);
        }
        msg.message = buf;
        msg.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
        msg.line = file.lines();
        return LINE;
    }

    abort();
    if(this->reply_stream != NULL) {
        // if we were printing from an M command from pronterface we need to send this back
        this->reply_stream->printf("Done printing file\r\n");
        this->reply_stream = NULL;
    }
    return DONE;
}

/*
bool Player::check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value)
{
	float new_slope = 0.0;
	bool is_cluster = false;
	Gcode *gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
	if (!gcode->has_m && gcode->has_g && gcode->g == 1) {
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
    if(file.size() == 0 || !playing_file) return false;
    p.played_lines = this->playing_lines = current_line();
    p.elapsed_secs = this->calculate_elapsed_secs();
    p.percent_complete = roundf(file.bytes() * 100.0F / file.size());
    p.filename = this->filename;
    return true;
}

void Player::restart_job()
{
    if(this->last_filename.empty()) return;
    printk("Job restarted: %s.\r\n", this->last_filename.c_str());
    this->play_command(this->last_filename, &(StreamOutput::NullStream));
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!this->playing_file) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    if (sources.top() != this) { // a tool change or other script is half way; pause at the next file line instead
        this->suspend_pending = true;
        stream->printf("Suspending after the running script...\n");
        return;
    }
    suspend_now(stream);
}

void Player::suspend_now(StreamOutput *stream)
{
    stream->printf("Suspending , waiting for queue to empty...\n");

    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THECONVEYOR.wait_for_idle();

    if(THEKERNEL->is_halted()) {
        printk("Suspend aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);
    sources.suspend();

    // save current XYZ position in WCS
    Robot::wcs_t mpos= THEROBOT.get_axis_position();
    Robot::wcs_t wpos= THEROBOT.mcs2wcs(mpos);
    saved_position[0]= std::get<X_AXIS>(wpos);
    saved_position[1]= std::get<Y_AXIS>(wpos);
    saved_position[2]= std::get<Z_AXIS>(wpos);

    // save current state
    THEROBOT.push_state();
    current_motion_mode = THEROBOT.get_current_motion_mode();

    run_script("after_suspend", nullptr, 0);

    printk("Suspended, resume to continue playing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if (this->suspend_pending) {
        this->suspend_pending = false;
        stream->printf("Suspend cancelled\n");
        return;
    }
    if(!THEKERNEL->is_suspending()) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        printk("Resume aborted by kill\n");
        THEROBOT.pop_state();
        sources.resume();
        return;
    }

    if (this->goto_line == 0 && current_motion_mode > 1) { // back to the arc mode the job was in
        char buf[8];
        snprintf(buf, sizeof(buf), "G%d", current_motion_mode - 1);
        gcode_dispatch.run_line(buf, &StreamOutput::NullStream);
    }

    THEROBOT.pop_state();

    if(THEKERNEL->is_halted()) {
        printk("Resume aborted by kill\n");
        sources.resume();
        return;
    }

	sources.resume();

    // the before_resume script moves back to the saved position; without it the position is not restored
    if (this->goto_line == 0 && !run_script("before_resume", saved_position, 3)) stream->printf("Warning: no before_resume script\n");

	stream->printf("Playing file resumed\n");
}