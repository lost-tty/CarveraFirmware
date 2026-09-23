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
#include "libs/Kernel.h"
#include "GcodeDispatch.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "ConfigValue.h"
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


#define ATC_AXIS 4

#define atc_checksum            	CHECKSUM("atc")
#define probe_checksum            	CHECKSUM("probe")
#define endstop_pin_checksum      	CHECKSUM("homing_endstop_pin")
#define debounce_ms_checksum      	CHECKSUM("homing_debounce_ms")
#define max_travel_mm_checksum    	CHECKSUM("homing_max_travel_mm")
#define homing_retract_mm_checksum  CHECKSUM("homing_retract_mm")
#define homing_rate_mm_s_checksum   CHECKSUM("homing_rate_mm_s")
#define action_mm_checksum      	CHECKSUM("action_mm")
#define action_rate_mm_s_checksum   CHECKSUM("action_rate_mm_s")

#define detector_switch_checksum    CHECKSUM("toolsensor")
#define detector_checksum           CHECKSUM("detector")
#define detect_pin_checksum			CHECKSUM("detect_pin")
#define detect_rate_mm_s_checksum	CHECKSUM("detect_rate_mm_s")
#define detect_travel_mm_checksum 	CHECKSUM("detect_travel_mm")

#define safe_z_checksum				CHECKSUM("safe_z_mm")
#define safe_z_empty_checksum		CHECKSUM("safe_z_empty_mm")
#define safe_z_offset_checksum		CHECKSUM("safe_z_offset_mm")
#define fast_z_rate_checksum		CHECKSUM("fast_z_rate_mm_m")
#define slow_z_rate_checksum		CHECKSUM("slow_z_rate_mm_m")
#define margin_rate_checksum		CHECKSUM("margin_rate_mm_m")

#define fast_rate_mm_m_checksum		CHECKSUM("fast_rate_mm_m")
#define slow_rate_mm_m_checksum		CHECKSUM("slow_rate_mm_m")
#define retract_mm_checksum			CHECKSUM("retract_mm")
#define probe_height_mm_checksum	CHECKSUM("probe_height_mm")

#define coordinate_checksum			CHECKSUM("coordinate")
#define anchor1_x_checksum			CHECKSUM("anchor1_x")
#define anchor1_y_checksum			CHECKSUM("anchor1_y")
#define anchor2_offset_x_checksum	CHECKSUM("anchor2_offset_x")
#define anchor2_offset_y_checksum	CHECKSUM("anchor2_offset_y")
#define rotation_offset_x_checksum	CHECKSUM("rotation_offset_x")
#define rotation_offset_y_checksum	CHECKSUM("rotation_offset_y")
#define rotation_offset_z_checksum	CHECKSUM("rotation_offset_z")
#define toolrack_offset_x_checksum	CHECKSUM("toolrack_offset_x")
#define toolrack_offset_y_checksum	CHECKSUM("toolrack_offset_y")
#define toolrack_z_checksum			CHECKSUM("toolrack_z")
#define clearance_x_checksum		CHECKSUM("clearance_x")
#define clearance_y_checksum		CHECKSUM("clearance_y")
#define clearance_z_checksum		CHECKSUM("clearance_z")

void ATCHandler::on_module_loaded()
{
	tool_detected = false;
    atc_home_info.clamp_status = UNHOMED;
    atc_home_info.triggered = false;
    detector_info.triggered = false;


    ADD_MCODE(m490, 490, BARRIER, ATCHandler::clamp_gcode);
    ADD_MCODE(m492, 492, BARRIER, ATCHandler::detect_gcode);
    ADD_MCODE(m493, 493, BARRIER, ATCHandler::tool_gcode);
    ADD_MCODE(m494, 494, IMMEDIATE, ATCHandler::probe_laser_gcode);
    ADD_MCODE(m497, 497, BARRIER, ATCHandler::state_gcode);

    this->on_config_reload(this);

    this->register_params();
    SimpleShell::add_command(shell_slot, "atc", &ATCHandler::shell, this, "atc [rack] - tool and clamp state, rack geometry");

	read_endstop_timer.start();
	read_detector_timer.start();
}

void ATCHandler::on_config_reload(void *argument)
{
	atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("1.0^" )->as_string())->as_input();
	atc_home_info.debounce_ms    = THEKERNEL->config->value(atc_checksum, debounce_ms_checksum)->by_default(1  )->as_number();
	atc_home_info.max_travel    = THEKERNEL->config->value(atc_checksum, max_travel_mm_checksum)->by_default(8  )->as_number();
	atc_home_info.retract    = THEKERNEL->config->value(atc_checksum, homing_retract_mm_checksum)->by_default(3  )->as_number();
	atc_home_info.action_dist    = THEKERNEL->config->value(atc_checksum, action_mm_checksum)->by_default(1  )->as_number();
	atc_home_info.homing_rate    = THEKERNEL->config->value(atc_checksum, homing_rate_mm_s_checksum)->by_default(1  )->as_number();
	atc_home_info.action_rate    = THEKERNEL->config->value(atc_checksum, action_rate_mm_s_checksum)->by_default(1  )->as_number();

	detector_info.detect_pin.from_string( THEKERNEL->config->value(atc_checksum, detector_checksum, detect_pin_checksum)->by_default("0.20^" )->as_string())->as_input();
	detector_info.detect_rate = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_rate_mm_s_checksum)->by_default(1  )->as_number();
	detector_info.detect_travel = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_travel_mm_checksum)->by_default(1  )->as_number();

	this->safe_z_mm = THEKERNEL->config->value(atc_checksum, safe_z_checksum)->by_default(-10)->as_number();
	this->safe_z_empty_mm = THEKERNEL->config->value(atc_checksum, safe_z_empty_checksum)->by_default(-20)->as_number();
	this->safe_z_offset_mm = THEKERNEL->config->value(atc_checksum, safe_z_offset_checksum)->by_default(10)->as_number();
	this->fast_z_rate = THEKERNEL->config->value(atc_checksum, fast_z_rate_checksum)->by_default(500)->as_number();
	this->slow_z_rate = THEKERNEL->config->value(atc_checksum, slow_z_rate_checksum)->by_default(60)->as_number();
	this->margin_rate = THEKERNEL->config->value(atc_checksum, margin_rate_checksum)->by_default(1000)->as_number();

	this->probe_fast_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, fast_rate_mm_m_checksum)->by_default(300  )->as_number();
	this->probe_slow_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, slow_rate_mm_m_checksum)->by_default(60   )->as_number();
	this->probe_retract_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, retract_mm_checksum)->by_default(2   )->as_number();
	this->probe_height_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, probe_height_mm_checksum)->by_default(0   )->as_number();

	this->anchor1_x = THEKERNEL->config->value(coordinate_checksum, anchor1_x_checksum)->by_default(-359  )->as_number();
	this->anchor1_y = THEKERNEL->config->value(coordinate_checksum, anchor1_y_checksum)->by_default(-234  )->as_number();
	this->anchor2_offset_x = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_x_checksum)->by_default(90  )->as_number();
	this->anchor2_offset_y = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_y_checksum)->by_default(45.65F  )->as_number();

	this->toolrack_z = THEKERNEL->config->value(coordinate_checksum, toolrack_z_checksum)->by_default(-105  )->as_number();
	this->toolrack_offset_x = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_x_checksum)->by_default(356  )->as_number();
	this->toolrack_offset_y = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_y_checksum)->by_default(0  )->as_number();

	probe_mx_mm = this->anchor1_x + this->toolrack_offset_x;
	probe_my_mm = this->anchor1_y + this->toolrack_offset_y + 180;
	probe_mz_mm = this->toolrack_z - 40;

	this->rotation_offset_x = THEKERNEL->config->value(coordinate_checksum, rotation_offset_x_checksum)->by_default(-8  )->as_number();
	this->rotation_offset_y = THEKERNEL->config->value(coordinate_checksum, rotation_offset_y_checksum)->by_default(37.5F  )->as_number();
	this->rotation_offset_z = THEKERNEL->config->value(coordinate_checksum, rotation_offset_z_checksum)->by_default(22.5F  )->as_number();

	this->clearance_x = THEKERNEL->config->value(coordinate_checksum, clearance_x_checksum)->by_default(-75  )->as_number();
	this->clearance_y = THEKERNEL->config->value(coordinate_checksum, clearance_y_checksum)->by_default(-3  )->as_number();
	this->clearance_z = THEKERNEL->config->value(coordinate_checksum, clearance_z_checksum)->by_default(-3  )->as_number();
}

void ATCHandler::cleanup()
{
    atc_state= 0;
    this->atc_home_info.clamp_status = UNHOMED;
}

// Called every millisecond in an ISR
void ATCHandler::read_endstop()
{

	if(!atc_homing || atc_home_info.triggered) return;

    if(THEROBOT.motor_is_moving(ATC_AXIS)) {
        // if it is moving then we check the probe, and debounce it
        if(atc_home_info.pin.get()) {
            if(debounce < atc_home_info.debounce_ms) {
                debounce++;
            } else {
            	THEROBOT.stop_motor(ATC_AXIS);
            	atc_home_info.triggered = true;
                debounce = 0;
            }

        } else {
            // The endstop was not hit yet
            debounce = 0;
        }
    }

    return;
}

// Called every millisecond in an ISR
void ATCHandler::read_detector()
{
    if(!detecting || detector_info.triggered) return;

    if (detector_info.detect_pin.get()) {
    	detector_info.triggered = true;
    }

    return;
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
    detecting = true;
    detector_info.triggered = false;

	float delta[Y_AXIS + 1] = {0};
	float half = detector_info.detect_travel / 2;
	delta[Y_AXIS] = half;
	if(!THEROBOT.delta_move_sync(delta, detector_info.detect_rate, Y_AXIS + 1)) return false;
	delta[Y_AXIS] = -detector_info.detect_travel;
	if(!THEROBOT.delta_move_sync(delta, detector_info.detect_rate, Y_AXIS + 1)) return false;
	delta[Y_AXIS] = half;
	if(!THEROBOT.delta_move_sync(delta, detector_info.detect_rate, Y_AXIS + 1)) return false;


	detecting = false;
	// switch off detector
	switch_state = false;
    ok = SwitchPool::set_state(detector_switch_checksum, switch_state);
    if (!ok) {
        printk("ERROR: Failed switch off detector switch.\r\n");
        return false;
    }

    // reset position
    THEROBOT.reset_position_from_current_actuator_position();

    return detector_info.triggered;
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
    debounce = 0;
    atc_homing = true;

    // home atc
	float delta[ATC_AXIS + 1] = {0};
	delta[ATC_AXIS] = atc_home_info.max_travel; // we go the max
	bool moved = THEROBOT.delta_move_sync(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	atc_homing = false;
	if(!moved) return;

    if (!atc_home_info.triggered) {
        THEKERNEL->halt(ATC_HOME_FAIL, "tool changer homing failed");
        printk("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    } else {
    	THEROBOT.reset_position_from_current_actuator_position();
    }

    // Move back
	delta[ATC_AXIS] = -atc_home_info.retract; // we go to retract position
	if(!THEROBOT.delta_move_sync(delta, atc_home_info.homing_rate, ATC_AXIS + 1)) return;

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
	delta[ATC_AXIS] = atc_home_info.action_dist;
	if(!THEROBOT.delta_move_sync(delta, atc_home_info.homing_rate, ATC_AXIS + 1)) return;

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
	delta[ATC_AXIS] = -atc_home_info.action_dist;
	if(!THEROBOT.delta_move_sync(delta, atc_home_info.action_rate, ATC_AXIS + 1)) return;

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
    THEKERNEL->halt(reason, msg);
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

const ATCHandler::Param ATCHandler::PARAMS[] = {
    {"_clamp_state", [](void *c) { return (float)((ATCHandler *)c)->atc_home_info.clamp_status; }},
    {"_tool_detected", [](void *c) { return (float)((ATCHandler *)c)->tool_detected; }},
    {"_active_tool", [](void *) { return (float)persist.tool(); }},
    {"_anchor1_x", [](void *c) { return (float)((ATCHandler *)c)->anchor1_x; }},
    {"_anchor1_y", [](void *c) { return (float)((ATCHandler *)c)->anchor1_y; }},
    {"_anchor2_offset_x", [](void *c) { return (float)((ATCHandler *)c)->anchor2_offset_x; }},
    {"_anchor2_offset_y", [](void *c) { return (float)((ATCHandler *)c)->anchor2_offset_y; }},
    {"_toolrack_offset_x", [](void *c) { return (float)((ATCHandler *)c)->toolrack_offset_x; }},
    {"_toolrack_offset_y", [](void *c) { return (float)((ATCHandler *)c)->toolrack_offset_y; }},
    {"_toolrack_z", [](void *c) { return (float)((ATCHandler *)c)->toolrack_z; }},
    {"_rotation_offset_x", [](void *c) { return (float)((ATCHandler *)c)->rotation_offset_x; }},
    {"_rotation_offset_y", [](void *c) { return (float)((ATCHandler *)c)->rotation_offset_y; }},
    {"_rotation_offset_z", [](void *c) { return (float)((ATCHandler *)c)->rotation_offset_z; }},
    {"_clearance_x", [](void *c) { return (float)((ATCHandler *)c)->clearance_x; }},
    {"_clearance_y", [](void *c) { return (float)((ATCHandler *)c)->clearance_y; }},
    {"_clearance_z", [](void *c) { return (float)((ATCHandler *)c)->clearance_z; }},
    {"_atc_safe_z", [](void *c) { return (float)((ATCHandler *)c)->safe_z_mm; }},
    {"_atc_safe_z_empty", [](void *c) { return (float)((ATCHandler *)c)->safe_z_empty_mm; }},
    {"_atc_safe_z_offset", [](void *c) { return (float)((ATCHandler *)c)->safe_z_offset_mm; }},
    {"_atc_fast_z_rate", [](void *c) { return (float)((ATCHandler *)c)->fast_z_rate; }},
    {"_atc_slow_z_rate", [](void *c) { return (float)((ATCHandler *)c)->slow_z_rate; }},
    {"_atc_margin_rate", [](void *c) { return (float)((ATCHandler *)c)->margin_rate; }},
    {"_atc_probe_fast_rate", [](void *c) { return (float)((ATCHandler *)c)->probe_fast_rate; }},
    {"_atc_probe_slow_rate", [](void *c) { return (float)((ATCHandler *)c)->probe_slow_rate; }},
    {"_atc_probe_retract", [](void *c) { return (float)((ATCHandler *)c)->probe_retract_mm; }},
    {"_atc_probe_height", [](void *c) { return (float)((ATCHandler *)c)->probe_height_mm; }},
    {"_probe_mx", [](void *c) { return (float)((ATCHandler *)c)->probe_mx_mm; }},
    {"_probe_my", [](void *c) { return (float)((ATCHandler *)c)->probe_my_mm; }},
    {"_probe_mz", [](void *c) { return (float)((ATCHandler *)c)->probe_mz_mm; }},
};

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
    stream->printf("ok\r\n");
}

void ATCHandler::sub_rack(std::string, StreamOutput *stream)
{
    for (int i = 0; i <= 6; i++) {
        stream->printf("tool%d  x %1.1f  y %1.1f  z %1.1f\r\n", i, anchor1_x + toolrack_offset_x,
                       anchor1_y + toolrack_offset_y + (i == 0 ? 210 : (6 - i) * 30), toolrack_z);
    }
    stream->printf("probe  x %1.1f  y %1.1f  z %1.1f\r\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
    stream->printf("ok\r\n");
}

void ATCHandler::register_params()
{
    static Parameters::Named slots[sizeof(PARAMS) / sizeof(*PARAMS)];
    for (unsigned i = 0; i < sizeof(PARAMS) / sizeof(*PARAMS); i++) Parameters::add(slots[i], PARAMS[i].name, PARAMS[i].get, this);
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
