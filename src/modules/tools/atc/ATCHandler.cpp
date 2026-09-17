/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ATCHandler.h"

#include <cstring>

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ATCHandler.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"
#include "modules/robot/Conveyor.h"
#include "libs/Logging.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "libs/utils.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "ZProbePublicAccess.h"
#include "SpindlePublicAccess.h"

#include "us_ticker_api.h"

#include "FileStream.h"
#include <math.h>

#define ATC_AXIS 4
#define STEPPER THEROBOT.actuators
// #define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

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
    ref_tool_mz = 0.0;
    cur_tool_mz = 0.0;
    tool_offset = 0.0;
    tool_number = 6;


    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);

    this->on_config_reload(this);

	read_endstop_timer.start();
	read_detector_timer.start();

    // load data from eeprom
    this->active_tool = THEKERNEL->eeprom_data.TOOL;
    this->ref_tool_mz = THEKERNEL->eeprom_data.REFMZ;
    this->cur_tool_mz = THEKERNEL->eeprom_data.TOOLMZ;
    this->tool_offset = THEKERNEL->eeprom_data.TLO;
}

void ATCHandler::on_config_reload(void *argument)
{
	char buff[10];

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

	atc_tools.clear();
	for (int i = 0; i <=  6; i ++) {
		struct atc_tool tool;
		tool.num = i;
	    // lift z axis to atc start position
		snprintf(buff, sizeof(buff), "tool%d", i);
		tool.mx_mm = this->anchor1_x + this->toolrack_offset_x;
		tool.my_mm = this->anchor1_y + this->toolrack_offset_y + (i == 0 ? 210 : (6 - i) * 30);
		tool.mz_mm = this->toolrack_z;
		atc_tools.push_back(tool);
	}
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

void ATCHandler::on_halt(void* argument)
{
    if (argument == nullptr ) {
        THEKERNEL->set_atc_state(ATC_NONE);
        this->atc_home_info.clamp_status = UNHOMED;
	}
}

// Called every millisecond in an ISR
void ATCHandler::read_endstop()
{

	if(!atc_homing || atc_home_info.triggered) return;

    if(STEPPER[ATC_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if(atc_home_info.pin.get()) {
            if(debounce < atc_home_info.debounce_ms) {
                debounce++;
            } else {
            	STEPPER[ATC_AXIS]->stop_moving();
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
		PublicData::set_value(atc_handler_checksum, set_wp_laser_checksum, nullptr);
	} else {
		probe_laser_timer.stop();
	}
}

bool ATCHandler::laser_detect() {
    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

    // switch on detector
    bool switch_state = true;
    bool ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
    if (!ok) {
        printk("ERROR: Failed switch on detector switch.\r\n");
        return false;
    }

    // move around and check laser detector
    detecting = true;
    detector_info.triggered = false;

	float delta[Y_AXIS + 1];
	for (size_t i = 0; i <= Y_AXIS; i++) delta[i] = 0;
	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT.delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= 0 - detector_info.detect_travel;
	THEROBOT.delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT.delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return false;


	detecting = false;
	// switch off detector
	switch_state = false;
    ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
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

    // get probe and calibrate states
    uint32_t probe_time;
    bool ok = PublicData::get_value(zprobe_checksum, get_zprobe_time_checksum, 0, &probe_time);
    if (ok) {
    	if (us_ticker_read() - probe_time < 5 * 1000 * 1000) {
    		return true;
    	}
    }

    return false;
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
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS]= atc_home_info.max_travel; // we go the max
	THEROBOT.delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	atc_homing = false;

    if (!atc_home_info.triggered) {
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
        printk("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    } else {
    	THEROBOT.reset_position_from_current_actuator_position();
    }

    // Move back
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS] = -atc_home_info.retract; // we go to retract position
	THEROBOT.delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return;

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

    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = atc_home_info.action_dist;
	THEROBOT.delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return;

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

	// First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = -atc_home_info.action_dist;
	THEROBOT.delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR.wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	// change clamp status
	atc_home_info.clamp_status = LOOSED;
	printk("ATC loosed!\r\n");
}

void ATCHandler::set_tool_offset()
{
    float px, py, pz;
    uint8_t ps;
    std::tie(px, py, pz, ps) = THEROBOT.get_last_probe_position();
    if (ps == 1) {
        cur_tool_mz = pz;
        if (ref_tool_mz < 0) {
        	tool_offset = cur_tool_mz - ref_tool_mz;
        	const float offset[3] = {0.0, 0.0, tool_offset};
        	THEROBOT.saveToolOffset(offset, cur_tool_mz);
        }
    }
	
}

void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
    	// gcode->stream->printf("Has m: %d\r\n", gcode->m);
		if (gcode->m == 490)  {
			if (gcode->subcode == 0) {
				// home tool change
				home_clamp();
			} else if (gcode->subcode == 1) {
				// clamp tool
				clamp_tool();
			} else if (gcode->subcode == 2) {
				// loose tool
				loose_tool();
			}
		} else if (gcode->m == 492) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// check true
				tool_detected = laser_detect();
				if (!tool_detected) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_NO_TOOL);
			        printk("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 2) {
				// check false
				tool_detected = laser_detect();
				if (tool_detected) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_HAS_TOOL);
			        printk("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 4) {
				tool_detected = laser_detect(); // a script decides what a wrong result means
			} else if (gcode->subcode == 3) {
				// check if the probe was triggered
				if (!probe_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(PROBE_INVALID);
			        printk("ERROR: Wireless probe dead or not set, please charge or set first!\n");
				}
			}
		} else if (gcode->m == 493) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// set tooll offset
				set_tool_offset();
			} else if (gcode->subcode == 2) {
				// set new tool
				if (gcode->has_letter('T')) {
		    		this->active_tool = gcode->get_value('T');
		    		// save current tool data to eeprom
		    		if (THEKERNEL->eeprom_data.TOOL != this->active_tool) {
		        	    THEKERNEL->eeprom_data.TOOL = this->active_tool;
		        	    THEKERNEL->write_eeprom_data();
		    		}

				} else {
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(ATC_NO_TOOL);
					printk("ERROR: No tool was set!\n");

				}
			}
		} else if (gcode->m == 494) {
			// control probe laser
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// open probe laser
				probe_laser_countdown = 120;
				probe_laser_timer.start();
			} else if (gcode->subcode == 2) {
				// close probe laser
				probe_laser_timer.stop();
			}
		} else if (gcode->m == 497) {
		    // wait for the queue to be empty
		    THECONVEYOR.wait_for_idle();
			THEKERNEL->set_atc_state(gcode->subcode);
		} else if (gcode->m == 498) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				printk("EEPRROM Data: TOOL:%d\n", THEKERNEL->eeprom_data.TOOL);
				printk("EEPRROM Data: TLO:%1.3f\n", THEKERNEL->eeprom_data.TLO);
				printk("EEPRROM Data: TOOLMZ:%1.3f\n", THEKERNEL->eeprom_data.TOOLMZ);
				printk("EEPRROM Data: REFMZ:%1.3f\n", THEKERNEL->eeprom_data.REFMZ);
				printk("EEPRROM Data: G54: %1.3f, %1.3f, %1.3f\n", THEKERNEL->eeprom_data.G54[0], THEKERNEL->eeprom_data.G54[1], THEKERNEL->eeprom_data.G54[2]);
			} else if (gcode->subcode == 2) {
				// Show EEPROM DATA
				THEKERNEL->erase_eeprom_data();
			}
		} else if ( gcode->m == 499 ) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				printk("tool:%d ref:%1.3f cur:%1.3f offset:%1.3f\n", active_tool, ref_tool_mz, cur_tool_mz, tool_offset);
			} else if (gcode->subcode == 2) {
				printk("probe -- mx:%1.1f my:%1.1f mz:%1.1f\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
				for (int i = 0; i <=  tool_number; i ++) {
					printk("tool%d -- mx:%1.1f my:%1.1f mz:%1.1f\n", atc_tools[i].num, atc_tools[i].mx_mm, atc_tools[i].my_mm, atc_tools[i].mz_mm);
				}
			}
		}
    }
}

void ATCHandler::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(get_param_checksum)) {
        struct atc_param *p = static_cast<struct atc_param *>(pdr->get_data_ptr());
        struct { const char *name; float value; } table[] = {
            {"_clamp_state", (float)atc_home_info.clamp_status}, {"_tool_detected", (float)tool_detected}, {"_active_tool", (float)active_tool},
            {"_anchor1_x", anchor1_x}, {"_anchor1_y", anchor1_y}, {"_anchor2_offset_x", anchor2_offset_x}, {"_anchor2_offset_y", anchor2_offset_y},
            {"_toolrack_offset_x", toolrack_offset_x}, {"_toolrack_offset_y", toolrack_offset_y}, {"_toolrack_z", toolrack_z},
            {"_rotation_offset_x", rotation_offset_x}, {"_rotation_offset_y", rotation_offset_y}, {"_rotation_offset_z", rotation_offset_z},
            {"_clearance_x", clearance_x}, {"_clearance_y", clearance_y}, {"_clearance_z", clearance_z},
            {"_atc_safe_z", safe_z_mm}, {"_atc_safe_z_empty", safe_z_empty_mm}, {"_atc_safe_z_offset", safe_z_offset_mm},
            {"_atc_fast_z_rate", fast_z_rate}, {"_atc_slow_z_rate", slow_z_rate}, {"_atc_margin_rate", margin_rate},
            {"_atc_probe_fast_rate", probe_fast_rate}, {"_atc_probe_slow_rate", probe_slow_rate}, {"_atc_probe_retract", probe_retract_mm},
            {"_atc_probe_height", probe_height_mm}, {"_probe_mx", probe_mx_mm}, {"_probe_my", probe_my_mm}, {"_probe_mz", probe_mz_mm},
        };
        for (auto &e : table) {
            if (strcmp(e.name, p->name) == 0) {
                p->value = e.value;
                pdr->set_taken();
                break;
            }
        }
    } else if(pdr->second_element_is(get_tool_status_checksum)) {
    	if (this->active_tool >= 0) {
            struct tool_status *t= static_cast<tool_status*>(pdr->get_data_ptr());
            t->active_tool = this->active_tool;
            t->ref_tool_mz = this->ref_tool_mz;
            t->cur_tool_mz = this->cur_tool_mz;
            t->tool_offset = this->tool_offset;
            pdr->set_taken();
    	}
    } else if (pdr->second_element_is(get_atc_pin_status_checksum)) {
        char *data = static_cast<char *>(pdr->get_data_ptr());
        // cover endstop
        data[0] = (char)this->atc_home_info.pin.get();
        data[1] = (char)this->detector_info.detect_pin.get();
        pdr->set_taken();
    }
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_ref_tool_mz_checksum)) {
        this->ref_tool_mz = cur_tool_mz;
        // update eeprom data if needed
        if (this->ref_tool_mz != THEKERNEL->eeprom_data.REFMZ) {
        	THEKERNEL->eeprom_data.REFMZ = this->ref_tool_mz;
		    THEKERNEL->write_eeprom_data();
        }
        this->tool_offset = 0.0;
        pdr->set_taken();
    }
}
