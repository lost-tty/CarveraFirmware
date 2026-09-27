/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ATCHandler.h"
#include "Persist.h"
#include "checksumm.h"
#include <cstring>
#include <math.h>
#include "libs/Kernel.h"
#include "GcodeDispatch.h"
#include "ConfigTable.h"
#include "ATCConfig.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "WirelessProbe.h"
#include "Gcode.h"
#include "libs/Logging.h"
#include "SwitchPool.h"
#include "ATCHandlerPublicAccess.h"
#include "utils/Parameters.h"
#include "SimpleShell.h"
#include "us_ticker_api.h"
#include "modules/robot/MachineTask.h"


#define ATC_AXIS 4

#define detector_switch_checksum    CHECKSUM("toolsensor")

static ATCConfigT atc_config;
static void atc_config_changed(const ConfigTable::Group *, const void *cfg)
{
    atc_config = *(const ATCConfigT *)cfg;
}
CONFIG_GROUPS(atc_config_groups,
    CFG_GROUP("atc", atc_root_config_keys, ATCConfigT, atc_config_changed),
    CFG_GROUP_MORE("atc.detector", atc_detector_config_keys, atc_config_changed),
    CFG_GROUP_MORE("atc.probe", atc_probe_config_keys, atc_config_changed),
    CFG_GROUP_MORE("coordinate", atc_coordinate_config_keys, atc_config_changed));

void ATCHandler::on_module_loaded()
{
	tool_detected = false;
    atc_home_info.clamp_status = UNHOMED;
    atc_home_info.triggered = false;


    ADD_MCODE(m490, 490, BARRIER, ATCHandler::clamp_gcode);
    ADD_MCODE(m492, 492, BARRIER, ATCHandler::detect_gcode);
    ADD_MCODE(m493, 493, BARRIER, ATCHandler::tool_gcode);
    ADD_MCODE(m494, 494, IMMEDIATE, ATCHandler::probe_laser_gcode);
    ADD_MCODE(m497, 497, BARRIER, ATCHandler::state_gcode);

    this->on_config_reload(this);

    this->register_params();
    SimpleShell::add_command(shell_slot, "atc", &ATCHandler::shell, this, "atc [rack] - tool and clamp state, rack geometry");
}

void ATCHandler::on_config_reload(void *argument)
{
    atc_config = ConfigTable::config<ATCConfigT>(atc_config_groups);
	atc_home_info.pin.from_spec(atc_config.homing_endstop_pin)->as_input();
	detector_info.detect_pin.from_spec(atc_config.detect_pin)->as_input();
}

float ATCHandler::probe_mx() { return atc_config.anchor1_x + atc_config.toolrack_offset_x; }
float ATCHandler::probe_my() { return atc_config.anchor1_y + atc_config.toolrack_offset_y + 180; }
float ATCHandler::probe_mz() { return atc_config.toolrack_z - 40; }

void ATCHandler::cleanup()
{
    atc_state= 0;
    this->atc_home_info.clamp_status = UNHOMED;
}

void ATCHandler::countdown_probe_laser()
{
	if (this->probe_laser_countdown > 0) {
		this->probe_laser_countdown--;
		wireless_probe.fire_laser();
	} else {
		probe_laser_timer.stop();
	}
}

bool ATCHandler::laser_detect() {
    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

    // switch on detector
    bool switch_state = true;
    bool ok = SwitchPool::set_state(detector_switch_checksum, switch_state);
    if (!ok) {
        printk("ERROR: Failed switch on detector switch.\r\n");
        return false;
    }

    // move around and check laser detector
    atc_watch.inputs.clear();
    atc_watch.witness.clear();
    atc_watch.inputs.add(detector_info.detect_pin);
    atc_watch.motors= 1 << Y_AXIS;
    atc_watch.hysteresis= 0;
    atc_watch.observe= true;

	float delta[Y_AXIS + 1] = {0};
	float half = atc_config.detect_travel_mm / 2;

	bool detected = false;
	delta[Y_AXIS] = half;
	if(!THEROBOT.delta_move_watch(delta, atc_config.detect_rate_mm_s, Y_AXIS + 1, atc_watch)) return false;
	detected |= atc_watch.hit;
	delta[Y_AXIS] = -atc_config.detect_travel_mm;
	if(!THEROBOT.delta_move_watch(delta, atc_config.detect_rate_mm_s, Y_AXIS + 1, atc_watch)) return false;
	detected |= atc_watch.hit;
	delta[Y_AXIS] = half;
	if(!THEROBOT.delta_move_watch(delta, atc_config.detect_rate_mm_s, Y_AXIS + 1, atc_watch)) return false;
	detected |= atc_watch.hit;

	// switch off detector
	switch_state = false;
    ok = SwitchPool::set_state(detector_switch_checksum, switch_state);
    if (!ok) {
        printk("ERROR: Failed switch off detector switch.\r\n");
        return false;
    }

    // reset position
    THEROBOT.reset_position_from_current_actuator_position();

    return detected;
}

bool ATCHandler::probe_detect() {
    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

    return us_ticker_read() - zprobe.getProbeTriggerTime() < 5 * 1000 * 1000;
}

void ATCHandler::home_clamp()
{
	printk("Homing atc...\n");
    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

    atc_home_info.triggered = false;
    atc_home_info.clamp_status = UNHOMED;

    atc_watch.inputs.clear();
    atc_watch.witness.clear();
    atc_watch.inputs.add(atc_home_info.pin);
    atc_watch.motors= 1 << ATC_AXIS;
    float steps_per_mm = THEROBOT.motor_steps_per_mm(ATC_AXIS);
    // the switch has to hold for debounce_ms at the homing rate, at least one step
    atc_watch.hysteresis= (uint16_t)ceilf(atc_config.homing_debounce_ms / 1000.0F * atc_config.homing_rate_mm_s
                                          * steps_per_mm);
    atc_watch.observe= false;

	float delta[ATC_AXIS + 1] = {0};
	delta[ATC_AXIS] = atc_config.homing_max_travel_mm; // we go the max
	bool moved = THEROBOT.delta_move_watch(delta, atc_config.homing_rate_mm_s, ATC_AXIS + 1, atc_watch);
	atc_home_info.triggered = atc_watch.hit;
	if(!moved) return;

    if (!atc_home_info.triggered) {
        machine_task.halt(ATC_HOME_FAIL, "tool changer homing failed");
        printk("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    } else {
    	THEROBOT.reset_position_from_current_actuator_position();
    }

    // the retract is measured from the switch edge, not from where the braking ended
	float past_edge = (THEROBOT.motor_step(ATC_AXIS) - atc_watch.at_steps[ATC_AXIS]) / steps_per_mm;
	delta[ATC_AXIS] = -atc_config.homing_retract_mm - past_edge;
	if(!THEROBOT.delta_move_sync(delta, atc_config.homing_rate_mm_s, ATC_AXIS + 1)) return;

	atc_home_info.clamp_status = CLAMPED;
	printk("ATC homed!\r\n");

}

void ATCHandler::clamp_tool()
{
	if (atc_home_info.clamp_status == CLAMPED) {
		printk("Already clamped!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp(); // homing ends at the clamped position, no stroke needed
		return;
	}

	THECONVEYOR.wait_for_idle(); // the spindle must have stopped moving before the clamp acts

	float delta[ATC_AXIS + 1] = {0};
	delta[ATC_AXIS] = atc_config.action_mm;
	if(!THEROBOT.delta_move_sync(delta, atc_config.homing_rate_mm_s, ATC_AXIS + 1)) return;

	// change clamp status
	atc_home_info.clamp_status = CLAMPED;
	printk("ATC clamped!\r\n");
}

void ATCHandler::loose_tool()
{
	if (atc_home_info.clamp_status == LOOSED) {
		printk("Already loosed!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp();
	}

	THECONVEYOR.wait_for_idle(); // the spindle must have stopped moving before the clamp acts

	float delta[ATC_AXIS + 1] = {0};
	delta[ATC_AXIS] = -atc_config.action_mm;
	if(!THEROBOT.delta_move_sync(delta, atc_config.action_rate_mm_s, ATC_AXIS + 1)) return;

	// change clamp status
	atc_home_info.clamp_status = LOOSED;
	printk("ATC loosed!\r\n");
}

void ATCHandler::set_tool_offset()
{
    float px, py, pz;
    uint8_t ps;
    std::tie(px, py, pz, ps) = THEROBOT.get_last_probe_position();
    if (ps != 1) return;

    float ref = persist.reference_z();
    const float offset[3] = {0.0, 0.0, ref < 0 ? pz - ref : 0.0};
    THEROBOT.saveToolOffset(offset, pz);
}

static void halt(int reason, const char *msg)
{
    machine_task.halt(reason, msg);
}

// M490: home the clamp, M490.1 clamp, M490.2 loosen
void ATCHandler::clamp_gcode(Gcode *gcode)
{
    switch (gcode->subcode) {
        case 0: home_clamp(); break;
        case 1: clamp_tool(); break;
        case 2: loose_tool(); break;
    }
}

// M492: is the slot occupied (.0 .1 expect a tool, .2 expects none, .4 only reports), M492.3 is the probe alive
void ATCHandler::detect_gcode(Gcode *gcode)
{
    switch (gcode->subcode) {
        case 0: case 1:
            tool_detected = laser_detect();
            if (!tool_detected) halt(ATC_NO_TOOL, "Tool confliction occured, please check tool rack!");
            break;
        case 2:
            tool_detected = laser_detect();
            if (tool_detected) halt(ATC_HAS_TOOL, "Tool confliction occured, please check tool rack!");
            break;
        case 3:
            if (!probe_detect()) halt(PROBE_INVALID, "Wireless probe dead or not set, please charge or set first!");
            break;
        case 4:
            tool_detected = laser_detect(); // a script decides what a wrong result means
            break;
    }
}

// M493: measure the tool length, M493.2 T<n> say which tool is in the spindle
void ATCHandler::tool_gcode(Gcode *gcode)
{
    switch (gcode->subcode) {
        case 0: case 1:
            set_tool_offset();
            break;
        case 2:
            if (!gcode->has_letter('T')) {
                halt(ATC_NO_TOOL, "No tool was set!");
                return;
            }
            persist.set_tool(gcode->get_value('T'));
            break;
    }
}

// M494: light the probe for two minutes, M494.2 turn it off
void ATCHandler::probe_laser_gcode(Gcode *gcode)
{
    switch (gcode->subcode) {
        case 0: case 1:
            probe_laser_countdown = 120;
            probe_laser_timer.start();
            break;
        case 2:
            probe_laser_timer.stop();
            break;
    }
}

// M497.<n>: what the status line reports as |A:<n> while a macro runs
void ATCHandler::state_gcode(Gcode *gcode)
{
    atc_state= gcode->subcode;
}

float ATCHandler::param_clamp_state(void *c)
{
    return (float)((ATCHandler *)c)->atc_home_info.clamp_status;
}
float ATCHandler::param_tool_detected(void *c) { return (float)((ATCHandler *)c)->tool_detected; }
float ATCHandler::param_active_tool(void *) { return (float)persist.tool(); }

template<float ATCConfigT::*M> static float config_param(void *) { return atc_config.*M; }
template<float (*F)()> static float derived_param(void *) { return F(); }

const SimpleShell::Sub<ATCHandler> ATCHandler::SUBS[] = {
    {"",      &ATCHandler::sub_state, "tool, offsets and clamp state"},
    {"rack",  &ATCHandler::sub_rack,  "where each slot and the probe sit"},
    {nullptr, nullptr, nullptr},
};

void ATCHandler::shell(void *self, const char *cmd, std::string args, StreamOutput *stream)
{
    SimpleShell::dispatch(static_cast<ATCHandler *>(self), SUBS, cmd, args, stream);
}

void ATCHandler::sub_state(std::string, StreamOutput *stream)
{
    stream->printf("tool %d, length offset %1.3f\r\n", persist.tool(), persist.tool_length());
    stream->printf("reference tool z %1.3f, current tool z %1.3f\r\n", persist.reference_z(), persist.tool_z());
    stream->printf("clamp %s, slot %s\r\n", atc_home_info.clamp_status == CLAMPED ? "clamped" :
                   atc_home_info.clamp_status == LOOSED ? "loosed" : "unhomed",
                   tool_detected ? "occupied" : "empty");
}

void ATCHandler::sub_rack(std::string, StreamOutput *stream)
{
    for (int i = 0; i <= 6; i++) {
        stream->printf("tool%d  x %1.1f  y %1.1f  z %1.1f\r\n", i,
                       atc_config.anchor1_x + atc_config.toolrack_offset_x,
                       atc_config.anchor1_y + atc_config.toolrack_offset_y
                           + (i == 0 ? 210 : (6 - i) * 30),
                       atc_config.toolrack_z);
    }
    stream->printf("probe  x %1.1f  y %1.1f  z %1.1f\r\n", probe_mx(), probe_my(), probe_mz());
}

void ATCHandler::register_params()
{
    static constexpr Parameters::Named rows[] = {
        {"_clamp_state", &ATCHandler::param_clamp_state},
        {"_tool_detected", &ATCHandler::param_tool_detected},
        {"_active_tool", &ATCHandler::param_active_tool},
        {"_anchor1_x", &config_param<&ATCConfigT::anchor1_x>},
        {"_anchor1_y", &config_param<&ATCConfigT::anchor1_y>},
        {"_anchor2_offset_x", &config_param<&ATCConfigT::anchor2_offset_x>},
        {"_anchor2_offset_y", &config_param<&ATCConfigT::anchor2_offset_y>},
        {"_toolrack_offset_x", &config_param<&ATCConfigT::toolrack_offset_x>},
        {"_toolrack_offset_y", &config_param<&ATCConfigT::toolrack_offset_y>},
        {"_toolrack_z", &config_param<&ATCConfigT::toolrack_z>},
        {"_rotation_offset_x", &config_param<&ATCConfigT::rotation_offset_x>},
        {"_rotation_offset_y", &config_param<&ATCConfigT::rotation_offset_y>},
        {"_rotation_offset_z", &config_param<&ATCConfigT::rotation_offset_z>},
        {"_clearance_x", &config_param<&ATCConfigT::clearance_x>},
        {"_clearance_y", &config_param<&ATCConfigT::clearance_y>},
        {"_clearance_z", &config_param<&ATCConfigT::clearance_z>},
        {"_atc_safe_z", &config_param<&ATCConfigT::safe_z_mm>},
        {"_atc_safe_z_empty", &config_param<&ATCConfigT::safe_z_empty_mm>},
        {"_atc_safe_z_offset", &config_param<&ATCConfigT::safe_z_offset_mm>},
        {"_atc_fast_z_rate", &config_param<&ATCConfigT::fast_z_rate_mm_m>},
        {"_atc_slow_z_rate", &config_param<&ATCConfigT::slow_z_rate_mm_m>},
        {"_atc_margin_rate", &config_param<&ATCConfigT::margin_rate_mm_m>},
        {"_atc_probe_fast_rate", &config_param<&ATCConfigT::probe_fast_rate_mm_m>},
        {"_atc_probe_slow_rate", &config_param<&ATCConfigT::probe_slow_rate_mm_m>},
        {"_atc_probe_retract", &config_param<&ATCConfigT::probe_retract_mm>},
        {"_atc_probe_height", &config_param<&ATCConfigT::probe_height_mm>},
        {"_probe_mx", &derived_param<&ATCHandler::probe_mx>},
        {"_probe_my", &derived_param<&ATCHandler::probe_my>},
        {"_probe_mz", &derived_param<&ATCHandler::probe_mz>},
    };
    Parameters::add(params_slot, rows, this);
}

bool ATCHandler::get_tool_status(struct tool_status *t) const
{
    if(persist.tool() < 0) return false;
    t->active_tool = persist.tool();
    t->ref_tool_mz = persist.reference_z();
    t->cur_tool_mz = persist.tool_z();
    t->tool_offset = persist.tool_length();
    return true;
}

void ATCHandler::get_pin_status(char *data) const
{
    data[0] = (char)this->atc_home_info.pin.get();
    data[1] = (char)this->detector_info.detect_pin.get();
}

// the current tool becomes the reference, so its offset is zero
void ATCHandler::set_ref_tool_mz()
{
    persist.set_reference_z(persist.tool_z());
    persist.set_tool_length(0);
}
