/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "GcodeDispatch.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/ActuatorCoordinates.h"
#include "Endstops.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Config.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "ScriptsPublicAccess.h"
#include "EndstopsPublicAccess.h"
#include "Logging.h"
#include "BaseSolution.h"
#include "SerialMessage.h"

#include <ctype.h>
#include <algorithm>

// OLD deprecated syntax
#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")

#define ENDSTOP_CHECKSUMS(X) {            \
    CHECKSUM(X "_min_endstop"),           \
    CHECKSUM(X "_max_endstop"),           \
    CHECKSUM(X "_max_travel"),            \
    CHECKSUM(X "_fast_homing_rate_mm_s"), \
    CHECKSUM(X "_slow_homing_rate_mm_s"), \
    CHECKSUM(X "_homing_retract_mm"),     \
    CHECKSUM(X "_homing_direction"),      \
    CHECKSUM(X "_min"),                   \
    CHECKSUM(X "_max"),                   \
    CHECKSUM(X "_limit_enable"),          \
	CHECKSUM(X "_motor_alarm_pin"),       \
}

// checksum defns
enum DEFNS { MIN_PIN, MAX_PIN, MAX_TRAVEL, FAST_RATE, SLOW_RATE, RETRACT, DIRECTION, MIN, MAX, LIMIT, ALARM_PIN, NDEFNS };

// global config settings

#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")
#define endstop_debounce_ms_checksum     CHECKSUM("endstop_debounce_ms")

#define home_z_first_checksum            CHECKSUM("home_z_first")
#define homing_order_checksum            CHECKSUM("homing_order")


// new config syntax
// endstop.xmin.enable true
// endstop.xmin.pin 1.29
// endstop.xmin.axis X
// endstop.xmin.homing_direction home_to_min

#define endstop_checksum                   CHECKSUM("endstop")
#define enable_checksum                    CHECKSUM("enable")
#define pin_checksum                       CHECKSUM("pin")
#define axis_checksum                      CHECKSUM("axis")
#define direction_checksum                 CHECKSUM("homing_direction")
#define position_checksum                  CHECKSUM("homing_position")
#define fast_rate_checksum                 CHECKSUM("fast_rate")
#define slow_rate_checksum                 CHECKSUM("slow_rate")
#define max_travel_checksum                CHECKSUM("max_travel")
#define retract_checksum                   CHECKSUM("retract")
#define limit_checksum                     CHECKSUM("limit_enable")
#define motor_alarm_checksum               CHECKSUM("limit_enable")

#define cover_endstop_checksum              CHECKSUM("cover_endstop")

#define STEPPER THEROBOT.actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())



// Homing States
enum STATES {
    MOVING_TO_ENDSTOP_FAST, // homing move
    MOVING_TO_ENDSTOP_SLOW, // homing move
    MOVING_BACK,            // homing move
    NOT_HOMING,
    BACK_OFF_HOME,
    LIMIT_TRIGGERED
};

void Endstops::on_module_loaded()
{
    this->status = NOT_HOMING;

    // Do not do anything if not enabled or if no pins are defined
    if (THEKERNEL->config->value( endstops_module_enable_checksum )->by_default(true)->as_bool()) {
        if(!load_old_config()) {
            return;
        }

    }else{
        // check for new config syntax
        if(!load_config()) {
            return;
        }
    }

    GcodeDispatch::add_handler(this);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);

	read_endstops_timer.start();

    // load g28 data from eeprom
//    this->g28_position[0] = THEKERNEL->eeprom_data.G28[0];
//    this->g28_position[1] = THEKERNEL->eeprom_data.G28[1];
//    this->g28_position[2] = THEKERNEL->eeprom_data.G28[2];
}

// Get config using old deprecated syntax Does not support ABC
bool Endstops::load_old_config()
{
    uint16_t const checksums[][NDEFNS] = {
        ENDSTOP_CHECKSUMS("alpha"),   // X
        ENDSTOP_CHECKSUMS("beta"),    // Y
        ENDSTOP_CHECKSUMS("gamma")    // Z
    };

    bool limit_enabled = false;
    for (int i = X_AXIS; i <= Z_AXIS; ++i) { // X_AXIS to Z_AXIS
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset = 0;
        hinfo.homed = false;
        hinfo.axis = 'X'+i;
        hinfo.axis_index = i;
        hinfo.pin_info = nullptr;

        // rates in mm/sec
        hinfo.fast_rate = THEKERNEL->config->value(checksums[i][FAST_RATE])->by_default(100)->as_number();
        hinfo.slow_rate = THEKERNEL->config->value(checksums[i][SLOW_RATE])->by_default(10)->as_number();

        // retract in mm
        hinfo.retract = THEKERNEL->config->value(checksums[i][RETRACT])->by_default(5)->as_number();

        // get homing direction and convert to boolean where true is home to min, and false is home to max
        hinfo.home_direction = THEKERNEL->config->value(checksums[i][DIRECTION])->by_default("home_to_min")->as_string() != "home_to_max";

        // homing cartesian position
        hinfo.homing_position = hinfo.home_direction ? THEKERNEL->config->value(checksums[i][MIN])->by_default(0)->as_number() : THEKERNEL->config->value(checksums[i][MAX])->by_default(200)->as_number();

        // used to set maximum movement on homing, set by alpha_max_travel if defined
        hinfo.max_travel = THEKERNEL->config->value(checksums[i][MAX_TRAVEL])->by_default(500)->as_number();

        // motor alarm info
        if (THEKERNEL->config->value(checksums[i][ALARM_PIN])->by_default("nc" )->as_string() != "nc") {
        	motor_alarm_info_t *info = new motor_alarm_info_t;
        	info->pin.from_string(THEKERNEL->config->value(checksums[i][ALARM_PIN])->as_string())->as_input();
            info->debounce = 0;
            info->axis = 'X' + i;
            info->axis_index = i;
            motor_alarms.push_back(info);
        }

        hinfo.motor_alarm_pin.from_string(THEKERNEL->config->value(checksums[i][ALARM_PIN])->by_default("nc" )->as_string())->as_input();

        // pin definitions for endstop pins
        for (int j = MIN_PIN; j <= MAX_PIN; ++j) {
            endstop_info_t *info = new endstop_info_t;
            info->pin.from_string(THEKERNEL->config->value(checksums[i][j])->by_default("nc" )->as_string())->as_input();
            if (!info->pin.connected()){
                // no pin defined try next
                delete info;
                continue;
            }

            // enter into endstop array
            endstops.push_back(info);

            // add index to the homing struct if this is the one used for homing
            if((hinfo.home_direction && j == MIN_PIN) || (!hinfo.home_direction && j == MAX_PIN)) hinfo.pin_info= info;

            // init struct
            info->debounce = 0;
            info->axis = 'X' + i;
            info->axis_index = i;

            // limits enabled
            info->limit_enable = THEKERNEL->config->value(checksums[i][LIMIT])->by_default(false)->as_bool();
            limit_enabled |= info->limit_enable;
        }

        homing_axis.push_back(hinfo);
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

    get_global_configs();

    if(limit_enabled) {
        register_for_event(ON_IDLE);
    }

    return true;
}

// Get config using new syntax supports ABC
bool Endstops::load_config()
{
    bool limit_enabled= false;
    size_t max_index= 0;

    std::array<homing_info_t, k_max_actuators> temp_axis_array; // needs to be at least XYZ, but allow for ABC
    {
        homing_info_t t;
        t.axis= 0;
        t.axis_index= 0;
        t.pin_info= nullptr;

        temp_axis_array.fill(t);
    }

    // iterate over all endstop.*.*
    std::vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, endstop_checksum);
    for(auto cs : modules ) {
        if(!THEKERNEL->config->value(endstop_checksum, cs, enable_checksum )->as_bool()) continue;

        endstop_info_t *pin_info= new endstop_info_t;
        pin_info->pin.from_string(THEKERNEL->config->value(endstop_checksum, cs, pin_checksum)->by_default("nc" )->as_string())->as_input();
        if(!pin_info->pin.connected()){
            // no pin defined try next
            delete pin_info;
            continue;
        }

        string axis= THEKERNEL->config->value(endstop_checksum, cs, axis_checksum)->by_default("")->as_string();
        if(axis.empty()){
            // axis is required
            delete pin_info;
            continue;
        }

        size_t i;
        switch(toupper(axis[0])) {
            case 'X': i= X_AXIS; break;
            case 'Y': i= Y_AXIS; break;
            case 'Z': i= Z_AXIS; break;
            case 'A': i= A_AXIS; break;
            case 'B': i= B_AXIS; break;
            case 'C': i= C_AXIS; break;
            default: // not a recognized axis
                delete pin_info;
                continue;
        }

        // check we are not going above the number of defined actuators/axis
        if(i >= THEROBOT.get_number_registered_motors()) {
            // too many axis we only have configured n_motors
            printk("ERROR: endstop %d is greater than number of defined motors. Endstops disabled\n", i);
            delete pin_info;
            return false;
        }

        // keep track of the maximum index that has been defined
        if(i > max_index) max_index= i;

        // init pin struct
        pin_info->debounce= 0;
        pin_info->axis= toupper(axis[0]);
        pin_info->axis_index= i;

        // are limits enabled
        pin_info->limit_enable= THEKERNEL->config->value(endstop_checksum, cs, limit_checksum)->by_default(false)->as_bool();
        limit_enabled |= pin_info->limit_enable;

        // enter into endstop array
        endstops.push_back(pin_info);

        // if set to none it means not used for homing (maybe limit only) so do not add to the homing array
        string direction= THEKERNEL->config->value(endstop_checksum, cs, direction_checksum)->by_default("none")->as_string();
        if(direction == "none") {
            continue;
        }

        // setup the homing array
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset= 0;
        hinfo.homed= false;
        hinfo.axis= toupper(axis[0]);
        hinfo.axis_index= i;
        hinfo.pin_info= pin_info;

        // rates in mm/sec
        hinfo.fast_rate= THEKERNEL->config->value(endstop_checksum, cs, fast_rate_checksum)->by_default(100)->as_number();
        hinfo.slow_rate= THEKERNEL->config->value(endstop_checksum, cs, slow_rate_checksum)->by_default(10)->as_number();

        // retract in mm
        hinfo.retract= THEKERNEL->config->value(endstop_checksum, cs, retract_checksum)->by_default(5)->as_number();

        // homing direction and convert to boolean where true is home to min, and false is home to max
        hinfo.home_direction=  direction == "home_to_min";

        // homing cartesian position
        hinfo.homing_position= THEKERNEL->config->value(endstop_checksum, cs, position_checksum)->by_default(hinfo.home_direction ? 0 : 200)->as_number();

        // used to set maximum movement on homing, set by max_travel if defined
        hinfo.max_travel= THEKERNEL->config->value(endstop_checksum, cs, max_travel_checksum)->by_default(500)->as_number();

        // stick into array in correct place
        temp_axis_array[hinfo.axis_index]= hinfo;
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    // copy to the homing_axis array, make sure that undefined entries are filled in as well
    // as the order is important and all slots must be filled upto the max_index
    for (size_t i = 0; i < temp_axis_array.size(); ++i) {
        if(temp_axis_array[i].axis == 0) {
            // was not configured above, if it is XYZ then we need to force a dummy entry
            if(i <= Z_AXIS) {
                homing_info_t t;
                t.axis= 'X' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);

            }else if(i <= max_index) {
                // for instance case where we defined C without A or B
                homing_info_t t;
                t.axis= 'A' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);
            }

        }else{
            homing_axis.push_back(temp_axis_array[i]);
        }
    }

    // saves some memory
    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

    // sets some endstop global configs applicable to all endstops
    get_global_configs();

    if(limit_enabled) {
        register_for_event(ON_IDLE);
    }

    return true;
}

void Endstops::get_global_configs()
{
    // NOTE the debounce count is in milliseconds so probably does not need to beset anymore
    this->debounce_ms= THEKERNEL->config->value(endstop_debounce_ms_checksum)->by_default(10)->as_number();
    this->debounce_count= THEKERNEL->config->value(endstop_debounce_count_checksum)->by_default(100)->as_number();


    this->home_z_first= THEKERNEL->config->value(home_z_first_checksum)->by_default(true)->as_bool();


	this->cover_endstop_pin.from_string( THEKERNEL->config->value(cover_endstop_checksum)->by_default("1.9^" )->as_string())->as_input();

    // see if an order has been specified, must be three or more characters, XYZABC or ABYXZ etc
    string order = THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order = 0;
    if(order.size() >= 3 && order.size() <= homing_axis.size()) {
        int shift = 0;
        for(auto c : order) {
            char n= toupper(c);
            uint32_t i = n >= 'X' ? n - 'X' : n - 'A' + 3;
            i += 1; // So X is 1
            if(i > 6) { // bad value
                this->homing_order = 0;
                break;
            }
            homing_order |= (i << shift);
            shift += 3;
        }
    }

    // set to true by default for deltas due to trim, false on cartesians
}

bool Endstops::debounced_get(Pin *pin)
{
    if(pin == nullptr) return false;
    uint32_t debounce = 0;
    while (pin->get()) {
        if ( ++debounce >= this->debounce_count ) {
            // pin triggered
            return true;
        }
    }
    return false;
}

// only called if limits are enabled
void Endstops::on_idle(void *argument)
{
    if(this->status == LIMIT_TRIGGERED) {
        // if we were in limit triggered see if it has been cleared
        for(auto& i : endstops) {
            if(i->limit_enable) {
                if(i->pin.get()) {
                    // still triggered, so exit
                    i->debounce = 0;
                    return;
                }

                if(i->debounce++ > debounce_count) { // can use less as it calls on_idle in between
                    // clear the state
                    this->status = NOT_HOMING;
                }
            }
        }
        return;

    } else if(this->status != NOT_HOMING) {
        // don't check while homing
        return;
    }

    if(THEKERNEL->is_halted()) return;

    for(auto& i : endstops) {
        if(i->limit_enable && STEPPER[i->axis_index]->is_moving()) {
            // check min and max endstops
            if(debounced_get(&i->pin)) {
                // endstop triggered
                printk("ALARM: Hard limit %c%c\n", STEPPER[i->axis_index]->which_direction() ? '-' : '+', i->axis);

                this->status = LIMIT_TRIGGERED;
                i->debounce = 0;
                // disables heaters and motors, ignores incoming Gcode and flushes block queue
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(HARD_LIMIT);
                return;
            }
        }
    }

    for(auto& i : motor_alarms) {
		// check min and max endstops
		if(debounced_get(&i->pin)) {
			// endstop triggered
			printk("ALARM: %c motor alarm triggered -  reset required\n", i->axis);

			i->debounce= 0;
			// disables heaters and motors, ignores incoming Gcode and flushes block queue
			THEKERNEL->call_event(ON_HALT, nullptr);
			THEKERNEL->set_halt_reason(MOTOR_ERROR_X + i->axis_index);
			return;
		}
	}
}

// if limit switches are enabled, then we must move off of the endstop otherwise we won't be able to move
// checks if triggered and only backs off if triggered
void Endstops::back_off_home(axis_bitmap_t axis)
{
    float delta[k_max_actuators]{0};
    bool moving= false;
    float slow_rate= NAN; // default mm/sec

    this->status = BACK_OFF_HOME;

    {
        // cartesians move every triggered axis off its endstop at once
        for( auto& e : homing_axis) {
            if(!axis[e.axis_index]) continue; // only for axes we asked to move
            if(e.pin_info == nullptr || !e.pin_info->limit_enable || !e.pin_info->triggered) continue;
            delta[e.axis_index]= e.retract * (e.home_direction ? 1 : -1);
            moving= true;
            // select slowest of them all
            slow_rate= isnan(slow_rate) ? e.slow_rate : std::min(slow_rate, e.slow_rate);
        }
    }

    if(moving) THEROBOT.delta_move_sync(delta, slow_rate, THEROBOT.get_number_registered_motors());

    this->status = NOT_HOMING;
}

// after homing X and Y the machine script may take over (after_home.ngc)
void Endstops::after_home(axis_bitmap_t axis)
{
    if(!axis[X_AXIS] || !axis[Y_AXIS]) return;
    struct script_call call{"after_home", nullptr, 0};
    PublicData::set_value(scripts_checksum, run_script_checksum, &call);
}

// Called every millisecond in an ISR
void Endstops::read_endstops()
{
    if(this->status != MOVING_TO_ENDSTOP_SLOW && this->status != MOVING_TO_ENDSTOP_FAST) return; // not doing anything we need to monitor for

    // check each homing endstop
    for(auto& e : homing_axis) { // check all axis homing endstops
        if(e.pin_info == nullptr) continue; // ignore if not a homing endstop
        int m= e.axis_index;

        if(STEPPER[m]->is_moving()) {
            // if it is moving then we check the associated endstop, and debounce it
            if(e.pin_info->pin.get()) {
                if(e.pin_info->debounce < debounce_ms) {
                    e.pin_info->debounce++;

                } else {
                    // we signal the motor to stop, which will preempt any moves on that axis
                    STEPPER[m]->stop_moving();
                    e.pin_info->triggered= true;
                }

            } else {
                // The endstop was not hit yet
                e.pin_info->debounce= 0;
            }
        }
    }

    return;
}

void Endstops::home_xy()
{
    if(axis_to_home[X_AXIS] && axis_to_home[Y_AXIS]) {
        // Home XY first so as not to slow them down by homing Z at the same time
        float delta[3] {homing_axis[X_AXIS].max_travel, homing_axis[Y_AXIS].max_travel, 0};
        if(homing_axis[X_AXIS].home_direction) delta[X_AXIS]= -delta[X_AXIS];
        if(homing_axis[Y_AXIS].home_direction) delta[Y_AXIS]= -delta[Y_AXIS];
        float feed_rate = std::min(homing_axis[X_AXIS].fast_rate, homing_axis[Y_AXIS].fast_rate);
        THEROBOT.delta_move(delta, feed_rate, 3);

    } else if(axis_to_home[X_AXIS]) {
        // now home X only
        float delta[3] {homing_axis[X_AXIS].max_travel, 0, 0};
        if(homing_axis[X_AXIS].home_direction) delta[X_AXIS]= -delta[X_AXIS];
        THEROBOT.delta_move(delta, homing_axis[X_AXIS].fast_rate, 3);

    } else if(axis_to_home[Y_AXIS]) {
        // now home Y only
        float delta[3] {0,  homing_axis[Y_AXIS].max_travel, 0};
        if(homing_axis[Y_AXIS].home_direction) delta[Y_AXIS]= -delta[Y_AXIS];
        THEROBOT.delta_move(delta, homing_axis[Y_AXIS].fast_rate, 3);
    }

    // Wait for axis to have homed
    THECONVEYOR.wait_for_idle();
}

void Endstops::home(axis_bitmap_t a)
{
    // reset debounce counts for all endstops
    for(auto& e : endstops) {
       e->debounce= 0;
       e->triggered= false;
    }

    this->axis_to_home= a;

    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;

    THEROBOT.disable_segmentation= true; // we must disable segmentation as this won't work with it enabled

    if(!home_z_first) home_xy();

    if(axis_to_home[Z_AXIS]) {
        // now home z
        float delta[3] {0, 0, homing_axis[Z_AXIS].max_travel}; // we go the max z
        if(homing_axis[Z_AXIS].home_direction) delta[Z_AXIS]= -delta[Z_AXIS];
        THEROBOT.delta_move(delta, homing_axis[Z_AXIS].fast_rate, 3);
        // wait for Z
        THECONVEYOR.wait_for_idle();
    }

    if(home_z_first) home_xy();

    // potentially home A B and C individually
    if(homing_axis.size() > 3){
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i]) {
                // now home A B or C
                float delta[i+1];
                for (size_t j = 0; j <= i; ++j) delta[j]= 0;
                delta[i]= homing_axis[i].max_travel; // we go the max
                if(homing_axis[i].home_direction) delta[i]= -delta[i];
                THEROBOT.delta_move(delta, homing_axis[i].fast_rate, i+1);
                // wait for it
                THECONVEYOR.wait_for_idle();
            }
        }
    }

    // check that the endstops were hit and it did not stop short for some reason
    // if the endstop is not triggered then enter ALARM state
    // with deltas we check all three axis were triggered, but at least one of XYZ must be set to home
    if(axis_to_home[X_AXIS] || axis_to_home[Y_AXIS] || axis_to_home[Z_AXIS]) {
        for (size_t i = X_AXIS; i <= Z_AXIS; ++i) {
            if(axis_to_home[i] && !homing_axis[i].pin_info->triggered) {
                this->status = NOT_HOMING;
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(HOME_FAIL);
                THEROBOT.disable_segmentation= false;
                return;
            }
        }
    }

    // also check ABC
    if(homing_axis.size() > 3){
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i] && !homing_axis[i].pin_info->triggered) {
                this->status = NOT_HOMING;
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(HOME_FAIL);
                THEROBOT.disable_segmentation = false;
                return;
            }
        }
    }

    // we did not complete movement the full distance if we hit the endstops
    // TODO Maybe only reset axis involved in the homing cycle
    THEROBOT.reset_position_from_current_actuator_position();

    // Move back a small distance for all homing axis
    this->status = MOVING_BACK;
    float delta[homing_axis.size()];
    for (size_t i = 0; i < homing_axis.size(); ++i) delta[i]= 0;

    // use minimum feed rate of all axes that are being homed (sub optimal, but necessary)
    float feed_rate= homing_axis[X_AXIS].slow_rate;
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract;
            if(!i.home_direction) delta[c]= -delta[c];
            feed_rate= std::min(i.slow_rate, feed_rate);
        }
    }

    THEROBOT.delta_move(delta, feed_rate, homing_axis.size());
    // wait until finished
    THECONVEYOR.wait_for_idle();

    // Start moving the axes towards the endstops slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract*2; // move further than we moved off to make sure we hit it cleanly
            if(i.home_direction) delta[c]= -delta[c];
        }else{
            delta[c]= 0;
        }
    }
    THEROBOT.delta_move(delta, feed_rate, homing_axis.size());
    // wait until finished
    THECONVEYOR.wait_for_idle();

    // we did not complete movement the full distance if we hit the endstops
    // TODO Maybe only reset axis involved in the homing cycle
    THEROBOT.reset_position_from_current_actuator_position();

    THEROBOT.disable_segmentation= false;

    this->status = NOT_HOMING;
}

void Endstops::process_home_command(Gcode* gcode)
{
    // First wait for the queue to be empty
    THECONVEYOR.wait_for_idle();

    // turn off any compensation transform so Z does not move as XY home
    auto savect= THEROBOT.compensationTransform;
    THEROBOT.compensationTransform= nullptr;

    // figure out which axis to home
    axis_bitmap_t haxis;
    haxis.reset();

    bool axis_speced = (gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ||
                        gcode->has_letter('A') || gcode->has_letter('B') || gcode->has_letter('C'));

    for (auto &p : homing_axis) {
        // only enable homing if the endstop is defined,
        if(p.pin_info == nullptr) continue;
        if(!axis_speced || gcode->has_letter(p.axis)) {
            haxis.set(p.axis_index);
            // now reset axis to 0 as we do not know what state we are in
            THEROBOT.reset_axis_position(0, p.axis_index);
        }
    }

    if(haxis.none()) {
        printk("WARNING: Nothing to home\n");
        return;
    }

    // do the actual homing
    if(homing_order != 0) {
        // if an order has been specified do it in the specified order
        // homing order is 0bfffeeedddcccbbbaaa where aaa is 1,2,3,4,5,6 to specify the first axis (XYZABC), bbb is the second and ccc is the third etc
        // eg 0b0101011001010 would be Y X Z A, 011 010 001 100 101 would be  B A X Y Z
        for (uint32_t m = homing_order; m != 0; m >>= 3) {
            uint32_t a= (m & 0x07)-1; // axis to home
            if(a < homing_axis.size() && haxis[a]) { // if axis is selected to home
                axis_bitmap_t bs;
                bs.set(a);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }

    } else {
        // they could all home at the same time
        home(haxis);
    }

    // restore compensationTransform
    THEROBOT.compensationTransform= savect;

    // check if on_halt (eg kill or fail)
    if(THEKERNEL->is_halted()) {
        printk("ALARM: Homing fail\n");
        // clear all the homed flags
        for (auto &p : homing_axis) p.homed= false;
        return;
    }

    // Zero the ax(i/e)s position, add in the home offset
    // NOTE that if compensation is active the Z will be set based on where XY are, so make sure XY are homed first then Z
    // so XY are at a known consistent position.  (especially true if using a proximity probe)
    for (auto &p : homing_axis) {
        if (haxis[p.axis_index]) { // if we requested this axis to home
            THEROBOT.reset_axis_position(p.homing_position + p.home_offset, p.axis_index);
            // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
            p.homed= true;
        }
    }

    // on some systems where 0,0 is bed center it is nice to have home goto 0,0 after homing
    // default is off for cartesian and on for deltas
    // if limit switches are enabled we must back off endstop after setting home
    back_off_home(haxis);
    after_home(haxis);
}

void Endstops::set_homing_offset(Gcode *gcode)
{
    // M306 Similar to M206 but sets Homing offsets based on current MCS position
    // Basically it finds the delta between the current MCS position and the requested position and adds it to the homing offset
    // then will not let it be set again until that axis is homed.
    float pos[3];
    THEROBOT.get_axis_position(pos);

    if (gcode->has_letter('X')) {
        if(!homing_axis[X_AXIS].homed) {
            gcode->stream->printf("error: Axis X must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[X_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('X')) - pos[X_AXIS]);
        homing_axis[X_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Y')) {
        if(!homing_axis[Y_AXIS].homed) {
            gcode->stream->printf("error: Axis Y must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Y_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('Y')) - pos[Y_AXIS]);
        homing_axis[Y_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Z')) {
        if(!homing_axis[Z_AXIS].homed) {
            gcode->stream->printf("error: Axis Z must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Z_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('Z')) - pos[Z_AXIS]);
        homing_axis[Z_AXIS].homed= false; // force it to be homed
    }

    gcode->stream->printf("Homing Offset: X %5.3f Y %5.3f Z %5.3f will take effect next home\n", homing_axis[X_AXIS].home_offset, homing_axis[Y_AXIS].home_offset, homing_axis[Z_AXIS].home_offset);
}


// parse gcodes
void Endstops::on_gcode_received(Gcode *argument)
{
    Gcode *gcode = argument;
    if ( gcode->has_g && gcode->g == 28) {
        switch(gcode->subcode) {
            case 0: // G28 in grbl mode will do a rapid to the predefined position otherwise it is home command
                // G28 goes to clearance via the g28 script
                break;

            case 1: // G28.1 set pre defined park position
                // saves current position in absolute machine coordinates
                THEROBOT.get_axis_position(g28_position); // Only XY are used
                // Note the following is only meant to be used for recovering a saved position from config-override
                // Not a standard Gcode and not to be relied on
                if (gcode->has_letter('X')) g28_position[X_AXIS] = gcode->get_value('X');
                if (gcode->has_letter('Y')) g28_position[Y_AXIS] = gcode->get_value('Y');

                // save g28 data to eeprom
//                THEKERNEL->eeprom_data.G28[0] = g28_position[X_AXIS];
//                THEKERNEL->eeprom_data.G28[1] = g28_position[Y_AXIS];
//                THEKERNEL->eeprom_data.G28[2] = g28_position[Z_AXIS];
//                THEKERNEL->write_eeprom_data();

                break;

            case 2: // G28.2 in grbl mode does homing (triggered by $H), otherwise it moves to the park position
                process_home_command(gcode);
                break;

            case 3: // G28.3 is a smoothie special it sets manual homing
                if(gcode->get_num_args() == 0) {
                    for (auto &p : homing_axis) {
                        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                        p.homed= true;
                        THEROBOT.reset_axis_position(0, p.axis_index);
                    }
                } else {
                    // do a manual homing based on given coordinates, no endstops required
                    if(gcode->has_letter('X')){ THEROBOT.reset_axis_position(gcode->get_value('X'), X_AXIS); homing_axis[X_AXIS].homed= true; }
                    if(gcode->has_letter('Y')){ THEROBOT.reset_axis_position(gcode->get_value('Y'), Y_AXIS); homing_axis[Y_AXIS].homed= true; }
                    if(gcode->has_letter('Z')){ THEROBOT.reset_axis_position(gcode->get_value('Z'), Z_AXIS); homing_axis[Z_AXIS].homed= true; }
                    if(homing_axis.size() > A_AXIS && homing_axis[A_AXIS].pin_info != nullptr && gcode->has_letter('A')){ THEROBOT.reset_axis_position(gcode->get_value('A'), A_AXIS); homing_axis[A_AXIS].homed= true; }
                    if(homing_axis.size() > B_AXIS && homing_axis[B_AXIS].pin_info != nullptr && gcode->has_letter('B')){ THEROBOT.reset_axis_position(gcode->get_value('B'), B_AXIS); homing_axis[B_AXIS].homed= true; }
                    if(homing_axis.size() > C_AXIS && homing_axis[C_AXIS].pin_info != nullptr && gcode->has_letter('C')){ THEROBOT.reset_axis_position(gcode->get_value('C'), C_AXIS); homing_axis[C_AXIS].homed= true; }
                }
                break;

            case 4: { // G28.4 is a smoothie special it sets manual homing based on the actuator position (used for rotary delta)
                    // do a manual homing based on given coordinates, no endstops required
                    ActuatorCoordinates ac{NAN, NAN, NAN};
                    if(gcode->has_letter('X')){ ac[0] =  gcode->get_value('X'); homing_axis[X_AXIS].homed= true; }
                    if(gcode->has_letter('Y')){ ac[1] =  gcode->get_value('Y'); homing_axis[Y_AXIS].homed= true; }
                    if(gcode->has_letter('Z')){ ac[2] =  gcode->get_value('Z'); homing_axis[Z_AXIS].homed= true; }
                    THEROBOT.reset_actuator_position(ac);
                }
                break;

            case 5: // G28.5 is a smoothie special it clears the homed flag for the specified axis, or all if not specifed
                if(gcode->get_num_args() == 0) {
                    for (auto &p : homing_axis) p.homed= false;
                } else {
                    if(gcode->has_letter('X')) homing_axis[X_AXIS].homed= false;
                    if(gcode->has_letter('Y')) homing_axis[Y_AXIS].homed= false;
                    if(gcode->has_letter('Z')) homing_axis[Z_AXIS].homed= false;
                    if(homing_axis.size() > A_AXIS && gcode->has_letter('A')) homing_axis[A_AXIS].homed= false;
                    if(homing_axis.size() > B_AXIS && gcode->has_letter('B')) homing_axis[B_AXIS].homed= false;
                    if(homing_axis.size() > C_AXIS && gcode->has_letter('C')) homing_axis[C_AXIS].homed= false;
                }
                break;

            case 6: // G28.6 is a smoothie special it shows the homing status of each axis
                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    gcode->stream->printf("%c:%d ", p.axis, p.homed);
                }
                gcode->add_nl= true;
                break;

            default:
                gcode->stream->printf("error:Unsupported command\n");
                break;
        }

    } else if (gcode->has_m) {

        switch (gcode->m) {
            case 119: {
                for(auto& h : homing_axis) {
                    if(h.pin_info == nullptr) continue; // ignore if not a homing endstop
                    string name;
                    name.append(1, h.axis).append(h.home_direction ? "_min" : "_max");
                    gcode->stream->printf("%s:%d ", name.c_str(), h.pin_info->pin.get());
                }
                gcode->stream->printf("pins- ");
                for(auto& p : endstops) {
                    string str(1, p->axis);
                    if(p->limit_enable) str.append("L");
                    gcode->stream->printf("(%s)P%d.%d:%d ", str.c_str(), p->pin.port_number, p->pin.pin, p->pin.get());
                }
                gcode->add_nl = true;
            }
            break;

            case 206: // M206 - set homing offset
                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    if (gcode->has_letter(p.axis)) p.home_offset= gcode->get_value(p.axis);
                }

                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    gcode->stream->printf("%c: %5.3f ", p.axis, p.home_offset);
                }

                gcode->stream->printf(" will take effect next home\n");
                break;

            case 306: // set homing offset based on current position

                set_homing_offset(gcode);
                break;

            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Home offset (mm):\nM206 ");
                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    gcode->stream->printf("%c%1.2f ", p.axis, p.home_offset);
                }
                gcode->stream->printf("\n");

                if(g28_position[X_AXIS] != 0 || g28_position[Y_AXIS] != 0) {
                    gcode->stream->printf(";predefined position:\nG28.1 X%1.4f Y%1.4f\n", g28_position[X_AXIS], g28_position[Y_AXIS]);
                }
                break;

        }
    }
}

void Endstops::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(home_offset_checksum)) {
        // provided by caller
        float *data = static_cast<float *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; ++i) {
            data[i]= homing_axis[i].home_offset;
        }
        pdr->set_taken();

    } else if(pdr->second_element_is(g28_position_checksum)) {
        pdr->set_data_ptr(&this->g28_position);
        pdr->set_taken();
    } else if(pdr->second_element_is(get_homing_status_checksum)) {
        bool *homing = static_cast<bool *>(pdr->get_data_ptr());
        *homing = this->status != NOT_HOMING;
        pdr->set_taken();

    } else if(pdr->second_element_is(get_homed_status_checksum)) {
        bool *homed = static_cast<bool *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; ++i) {
            homed[i]= homing_axis[i].homed;
        }
        pdr->set_taken();
    } else if (pdr->second_element_is(get_endstop_states_checksum)) {
    	int index = 0;
        char *data = static_cast<char *>(pdr->get_data_ptr());
        for(auto& i : endstops) {
        	if (index < 6) {
                if(i->limit_enable) {
                	data[index] = (char)i->pin.get();
                	index ++;
                }
        	}
        }
        // cover endstop
        data[5] = (char)this->cover_endstop_pin.get();
        pdr->set_taken();
    } else if (pdr->second_element_is(get_cover_endstop_state_checksum)) {
        bool *cover_state = static_cast<bool *>(pdr->get_data_ptr());
        *cover_state = this->cover_endstop_pin.get();
        pdr->set_taken();
    }
}

void Endstops::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(home_offset_checksum)) {
        float *t = static_cast<float*>(pdr->get_data_ptr());
        if(!isnan(t[0])) homing_axis[0].home_offset= t[0];
        if(!isnan(t[1])) homing_axis[1].home_offset= t[1];
        if(!isnan(t[2])) homing_axis[2].home_offset= t[2];
    }
}
