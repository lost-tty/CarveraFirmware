/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ZProbe.h"
#include "SimpleShell.h"

#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "Logging.h"
#include "Gcode.h"
#include "GcodeDispatch.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SerialMessage.h"
#include "ZProbePublicAccess.h"
#include "LevelingStrategy.h"
#include "utils.h"
#include "mbed.h"
#include "StreamOutput.h"

// strategies we know about
#include "ThreePointStrategy.h"
#include "CartGridStrategy.h"

#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define calibrate_pin_checksum   CHECKSUM("calibrate_pin")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")
#define return_feedrate_checksum CHECKSUM("return_feedrate")
#define probe_height_checksum    CHECKSUM("probe_height")
#define gamma_max_checksum       CHECKSUM("gamma_max")
#define max_z_checksum           CHECKSUM("max_z")
#define reverse_z_direction_checksum CHECKSUM("reverse_z")
#define dwell_before_probing_checksum CHECKSUM("dwell_before_probing")

// from endstop section

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPS_PER_MM(a) (THEROBOT.motor_steps_per_mm(a))
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

void ZProbe::on_module_loaded()
{
    invert_override = false;
    invert_probe = false;
    
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(true)->as_bool()) {
        return;
    }

    // load settings
    this->config_load();
    // register event-handlers
    GcodeDispatch::add_handler(this);
    Settings::add(settings_slot, &ZProbe::report_settings, this);
    for(auto s : strategies) s->register_mcodes();
    ADD_MCODE(m670, 670, IMMEDIATE, ZProbe::set_probe_settings);

    // we read the probe in this timer
    this->probe_trigger_time = 0;

}

void ZProbe::config_load()
{
    this->probe_pin.from_string( THEKERNEL->config->value(zprobe_checksum, probe_pin_checksum)->by_default("2.6v" )->as_string())->as_input();
    this->calibrate_pin.from_string( THEKERNEL->config->value(zprobe_checksum, calibrate_pin_checksum)->by_default("0.5^" )->as_string())->as_input();

    // get strategies to load
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, leveling_strategy_checksum);
    for( auto cs : modules ){
        if( THEKERNEL->config->value(leveling_strategy_checksum, cs, enable_checksum )->as_bool() ){
            bool found= false;
            LevelingStrategy *ls= nullptr;

            // check with each known strategy and load it if it matches
            switch(cs) {
                case three_point_leveling_strategy_checksum:
                    ls= new ThreePointStrategy(this);
                    found= true;
                    break;

                case cart_grid_leveling_strategy_checksum:
                    ls= new CartGridStrategy(this);
                    found= true;
                    break;
            }
            if(found) {
                // both strategies claim M561 and M565, so only one can be loaded
                if(!this->strategies.empty()) {
                    printk("WARNING: leveling strategy already loaded, ignoring the rest\n");
                    delete ls;
                } else if(ls->handleConfig()) {
                    this->strategies.push_back(ls);
                }else{
                    delete ls;
                }
            }
        }
    }

    this->probe_height  = THEKERNEL->config->value(zprobe_checksum, probe_height_checksum)->by_default(5)->as_number();
    this->slow_feedrate = THEKERNEL->config->value(zprobe_checksum, slow_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->fast_feedrate = THEKERNEL->config->value(zprobe_checksum, fast_feedrate_checksum)->by_default(100)->as_number(); // feedrate in mm/sec
    this->return_feedrate = THEKERNEL->config->value(zprobe_checksum, return_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->reverse_z     = THEKERNEL->config->value(zprobe_checksum, reverse_z_direction_checksum)->by_default(false)->as_bool(); // Z probe moves in reverse direction
    this->max_z         = THEKERNEL->config->value(zprobe_checksum, max_z_checksum)->by_default(NAN)->as_number(); // maximum zprobe distance
    if(isnan(this->max_z)){
        this->max_z = THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number(); // maximum zprobe distance
    }
    this->dwell_before_probing = THEKERNEL->config->value(zprobe_checksum, dwell_before_probing_checksum)->by_default(0)->as_number(); // dwell time in seconds before probing

}

// single probe in Z with custom feedrate
// returns boolean value indicating if probe was triggered
bool ZProbe::run_probe(float& mm, float feedrate, float max_dist, bool reverse)
{
    if(dwell_before_probing > .0001F) safe_delay_ms(dwell_before_probing*1000);

    if(this->probe_pin.get() != invert_probe) {
    	printk("Error: Probe already triggered so aborts\r\n");
        // probe already triggered so abort
        return false;
    }
    float maxz= max_dist < 0 ? this->max_z*2 : max_dist;

    probe_watch.inputs.clear();
    probe_watch.witness.clear();
    probe_watch.inputs.add(probe_pin, invert_probe);
    probe_watch.motors= (1<<X_AXIS)|(1<<Y_AXIS)|(1<<Z_AXIS);
    probe_watch.hysteresis= 0;   // a probe stops on the first edge, the measurement is that point

    int32_t z_start_steps= THEROBOT.motor_step(Z_AXIS);

    // move Z down
    bool dir= (!reverse_z != reverse); // xor
    float delta[3]= {0,0,0};
    delta[Z_AXIS]= dir ? -maxz : maxz;
    if(!THEROBOT.delta_move_watch(delta, feedrate, 3, probe_watch)) return false;

    int32_t at= probe_watch.hit ? probe_watch.at_steps[Z_AXIS] : THEROBOT.motor_step(Z_AXIS);
    mm = (z_start_steps - at) / THEROBOT.motor_steps_per_mm(Z_AXIS);

    THEROBOT.set_last_probe_position(std::make_tuple(0, 0, mm, probe_watch.hit ? 1:0));

    if(probe_watch.hit) {
        // the probe stopped the move, so the planner's idea of where we are is wrong
        THEROBOT.reset_position_from_current_actuator_position();
    }

    return probe_watch.hit;
}

// do probe then return to start position
bool ZProbe::run_probe_return(float& mm, float feedrate, float max_dist, bool reverse)
{
    float save_z_pos= THEROBOT.get_axis_position(Z_AXIS);

    bool ok= run_probe(mm, feedrate, max_dist, reverse);

    // move probe back to where it was
    float fr;
    if(this->return_feedrate != 0) { // use return_feedrate if set
        fr = this->return_feedrate;
    } else {
        fr = this->slow_feedrate*2; // nominally twice slow feedrate
        if(fr > this->fast_feedrate) fr = this->fast_feedrate; // unless that is greater than fast feedrate
    }

    // absolute move back to saved starting position
    coordinated_move(NAN, NAN, save_z_pos, fr, false);

    return ok;
}

bool ZProbe::doProbeAt(float &mm, float x, float y)
{
    // move to xy
    coordinated_move(x, y, NAN, getFastFeedrate() * 4);
    return run_probe_return(mm, slow_feedrate);
}

void ZProbe::report_settings(void *self, StreamOutput *stream)
{
    ZProbe *z= (ZProbe *)self;
    stream->printf(";Probe feedrates Slow/fast(K)/Return (mm/sec) max_z (mm) height (mm) dwell (s):\nM670 S%1.2f K%1.2f R%1.2f Z%1.2f H%1.2f D%1.2f\n",
        z->slow_feedrate, z->fast_feedrate, z->return_feedrate, z->max_z, z->probe_height, z->dwell_before_probing);

    for(auto s : z->strategies) s->report_settings(stream);
}

void ZProbe::on_gcode_received(Gcode *argument)
{
    Gcode *gcode = argument;

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 32) {

        invert_probe = false;
        // make sure the probe is defined and not already triggered before moving motors
        if(!this->probe_pin.connected()) {
            gcode->stream->printf("ZProbe pin not configured.\n");
            return;
        }

        // first wait for all moves to finish
        THECONVEYOR.wait_for_idle();

        if(this->probe_pin.get()) {
            gcode->stream->printf("ZProbe triggered before move, aborting command.\n");
            return;
        }

        if( gcode->g == 30 ) { // simple Z probe
            bool set_z= gcode->has_letter('Z');
            bool probe_result;
            bool reverse= (gcode->has_letter('R') && gcode->get_value('R') != 0); // specify to probe in reverse direction
            float rate= gcode->has_letter('F') ? gcode->get_value('F') / 60 : this->slow_feedrate;
            float mm;

            // if not setting Z then return probe to where it started, otherwise leave it where it is
            probe_result = (set_z ? run_probe(mm, rate, -1, reverse) : run_probe_return(mm, rate, -1, reverse));

            if(probe_result) {
                // the result is in actuator coordinates moved
                gcode->stream->printf("Z:%1.4f\n", THEROBOT.from_millimeters(mm));

                if(set_z) {
                    // set current Z to the specified value, shortcut for G92 Znnn
                    char buf[32];
                    snprintf(buf, sizeof(buf), "G92 Z%f", gcode->get_value('Z'));
                    gcode_dispatch.run_line(buf, &StreamOutput::NullStream);
                }

            } else {
                gcode->stream->printf("ZProbe not triggered\n");
            }

        } else {
            if(!gcode->has_letter('P')) {
                // find the first strategy to handle the gcode
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
                gcode->stream->printf("No strategy found to handle G%d\n", gcode->g);

            }else{
                // P paramater selects which strategy to send the code to
                // they are loaded in the order they are defined in config, 0 being the first, 1 being the second and so on.
                uint16_t i= gcode->get_value('P');
                if(i < strategies.size()) {
                    if(!strategies[i]->handleGcode(gcode)){
                        gcode->stream->printf("strategy #%d did not handle G%d\n", i, gcode->g);
                    }
                    return;

                }else{
                    gcode->stream->printf("strategy #%d is not loaded\n", i);
                }
            }
        }

    } else if(gcode->has_g && gcode->g == 38 ) { // G38.2 Straight Probe with error, G38.3 straight probe without error
        // linuxcnc/grbl style probe http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G38-probe
        if(gcode->subcode < 2 || gcode->subcode > 6) {
            gcode->stream->printf("Error :Only G38.2 to G38.5 are supported\n");
            return;
        }

        // make sure the probe is defined and not already triggered before moving motors
        if(!this->probe_pin.connected()) {
            gcode->stream->printf("Error :ZProbe not connected.\n");
            return;
        }

        if (gcode->subcode == 4 || gcode->subcode == 5) {
            invert_probe = true;
        } else {
            invert_probe = false;
        }

        if (gcode->subcode == 6) {
            calibrate_Z(gcode);
        } else {
            probe_XYZ(gcode);
        }

        invert_probe = false;

        return;

    }
}

// M670: the probe feedrates and limits
void ZProbe::set_probe_settings(Gcode *gcode)
{
    if (gcode->has_letter('S')) this->slow_feedrate = gcode->get_value('S');
    if (gcode->has_letter('K')) this->fast_feedrate = gcode->get_value('K');
    if (gcode->has_letter('R')) this->return_feedrate = gcode->get_value('R');
    if (gcode->has_letter('Z')) this->max_z = gcode->get_value('Z');
    if (gcode->has_letter('H')) this->probe_height = gcode->get_value('H');
    if (gcode->has_letter('D')) this->dwell_before_probing = gcode->get_value('D');
}

// special way to probe in the X or Y or Z direction using planned moves, should work with any kinematics
void ZProbe::probe_XYZ(Gcode *gcode)
{
    float x= 0, y= 0, z= 0;
    if(gcode->has_letter('X')) {
        x= gcode->get_value('X');
    }

    if(gcode->has_letter('Y')) {
        y= gcode->get_value('Y');
    }

    if(gcode->has_letter('Z')) {
        z= gcode->get_value('Z');
    }

    if(x == 0 && y == 0 && z == 0) {
        gcode->stream->printf("error:at least one of X Y or Z must be specified, and be > or < 0\n");
        return;
    }

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F')/60 : this->slow_feedrate;

    // first wait for all moves to finish
    THECONVEYOR.wait_for_idle();

    if(this->probe_pin.get() != invert_probe) {
        gcode->stream->printf("Error:ZProbe triggered before move, aborting command.\n");
        THEKERNEL->halt(PROBE_FAIL, "probe failed");
        return;
    }

    probe_watch.inputs.clear();
    probe_watch.inputs.add(probe_pin, invert_probe);
    probe_watch.motors= (1<<X_AXIS)|(1<<Y_AXIS)|(1<<Z_AXIS);
    probe_watch.hysteresis= 0;
    probe_watch.witness.clear();

    float delta[3]= {x, y, z};
    if(!THEROBOT.delta_move_watch(delta, rate, 3, probe_watch)) {
        if(!THEKERNEL->is_halted()) {
            gcode->stream->printf("ERROR: Move too small,  %1.3f, %1.3f, %1.3f\n", x, y, z);
            THEKERNEL->halt(PROBE_FAIL, "probe failed");
        }
        return;
    }

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT.reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT.get_axis_position(pos, 3);

    uint8_t probeok= probe_watch.hit ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", THEROBOT.from_millimeters(pos[X_AXIS]), THEROBOT.from_millimeters(pos[Y_AXIS]), THEROBOT.from_millimeters(pos[Z_AXIS]), probeok);
    THEROBOT.set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], probeok));

    if(probeok == 0 && (gcode->subcode == 2 || gcode->subcode == 4)) {
        // issue error if probe was not triggered and subcode is 2 or 4
        gcode->stream->printf("ALARM: Probe fail\n");
        THEKERNEL->halt(PROBE_FAIL, "probe failed");
    }
}

// just probe / calibrate Z using calibrate pin
void ZProbe::calibrate_Z(Gcode *gcode)
{
    float z= 0;
    if(gcode->has_letter('Z')) {
        z= gcode->get_value('Z');
    }

    if(z == 0) {
        gcode->stream->printf("error: Z must be specified, and be > or < 0\n");
        return;
    }

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F') / 60 : this->slow_feedrate;

    // first wait for all moves to finish
    THECONVEYOR.wait_for_idle();

    if (this->calibrate_pin.get()) {
        gcode->stream->printf("error: ZCalibrate triggered before move, aborting command.\n");
        return;
    }

    // a probe stuck on would look alive to M492.3 below, so remember whether it was quiet before the move
    bool probe_idle= this->probe_pin.get() == invert_probe;

    probe_watch.inputs.clear();
    probe_watch.inputs.add(calibrate_pin);
    // a live wireless probe signals as the setter does, which M492.3 reads below
    probe_watch.witness.clear();
    probe_watch.witness.add(probe_pin, invert_probe);
    probe_watch.motors= (1<<X_AXIS)|(1<<Y_AXIS)|(1<<Z_AXIS);
    probe_watch.hysteresis= 0;

    float delta[3]= {0, 0, z};
    if(!THEROBOT.delta_move_watch(delta, rate, 3, probe_watch)) {
        if(!THEKERNEL->is_halted()) {
            gcode->stream->printf("ERROR: Move too small,  %1.3f\n", z);
            THEKERNEL->halt(PROBE_FAIL, "probe failed");
        }
        return;
    }

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT.reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT.get_axis_position(pos, 3);

    uint8_t calibrateok = probe_watch.hit ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", THEROBOT.from_millimeters(pos[X_AXIS]), THEROBOT.from_millimeters(pos[Y_AXIS]), THEROBOT.from_millimeters(pos[Z_AXIS]), calibrateok);
    THEROBOT.set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], calibrateok));

    if (calibrateok == 0) {
        // issue error if probe was not triggered and subcode is 2 or 4
        gcode->stream->printf("ALARM: Calibrate fail!\n");
        THEKERNEL->halt(CALIBRATE_FAIL, "calibration failed");
    }

    // M492.3 reads this as "the wireless probe is alive": only a probe that signalled is
    if (probe_watch.hit && probe_watch.witnessed && probe_idle) {
    	this->probe_trigger_time = us_ticker_read();
    }

}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative)
{
    float delta[3]{0, 0, 0};
    if(relative) {
        if(!isnan(x)) delta[X_AXIS]= x;
        if(!isnan(y)) delta[Y_AXIS]= y;
        if(!isnan(z)) delta[Z_AXIS]= z;
    } else {
        // machine coordinates, ignoring any WCS offset
        float pos[3];
        THEROBOT.get_real_machine_position(pos);
        if(!isnan(x)) delta[X_AXIS]= x - pos[X_AXIS];
        if(!isnan(y)) delta[Y_AXIS]= y - pos[Y_AXIS];
        if(!isnan(z)) delta[Z_AXIS]= z - pos[Z_AXIS];
    }

    THEROBOT.delta_move_sync(delta, feedrate, 3);
}

// issue home command
void ZProbe::home()
{
    gcode_dispatch.run_line("G28.2", &StreamOutput::NullStream);
}

