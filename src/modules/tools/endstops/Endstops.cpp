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
#include "checksumm.h"
#include "utils.h"
#include "libs/StreamOutput.h"
#include "Scripts.h"
#include "Logging.h"
#include "ZProbe.h"
#include "BaseSolution.h"
#include "SerialMessage.h"
#include "EndstopsConfig.h"

#include <ctype.h>
#include <algorithm>
#include <cmath>
#include "modules/robot/MachineTask.h"

// All axes home to max. Z has no min endstop.
#define AXIS(min_pin, max_pin, travel, fast, alarm) \
    CFG_SET(EndstopAxisConfigT, min_endstop, min_pin), \
    CFG_SET(EndstopAxisConfigT, max_endstop, max_pin), \
    CFG_SET(EndstopAxisConfigT, max_travel, travel), \
    CFG_SET(EndstopAxisConfigT, fast_homing_rate, fast), \
    CFG_SET(EndstopAxisConfigT, motor_alarm_pin, alarm)
static const ConfigTable::Override alpha_ov[] = { AXIS("0.24^", "0.25^", 500.0f, 15.0f, "0.1!^") };
static const ConfigTable::Override beta_ov[] = { AXIS("1.1^", "1.4^", 380.0f, 15.0f, "0.0!^") };
static const ConfigTable::Override gamma_ov[] = { AXIS("nc", "1.8^", 150.0f, 10.0f, "3.25!^") };
#undef AXIS

static void endstops_config_changed(const ConfigTable::Group *, const void *)
{
    endstops.configure();
}
CONFIG_GROUPS(endstops_config_groups,
    CFG_GROUP_OV("alpha_", endstop_axis_config_keys, EndstopAxisConfigT, alpha_ov,
                 endstops_config_changed),
    CFG_GROUP_OV("beta_", endstop_axis_config_keys, EndstopAxisConfigT, beta_ov,
                 endstops_config_changed),
    CFG_GROUP_OV("gamma_", endstop_axis_config_keys, EndstopAxisConfigT, gamma_ov,
                 endstops_config_changed),
    CFG_GROUP("", endstops_global_config_keys, EndstopsGlobalConfigT, endstops_config_changed));

static const EndstopAxisConfigT &axis_config(int i)
{
    return ConfigTable::config<EndstopAxisConfigT>(&endstops_config_groups[i]);
}
static const EndstopsGlobalConfigT &global_config()
{
    return ConfigTable::config<EndstopsGlobalConfigT>(&endstops_config_groups[3]);
}

// Pins and homing direction are set once in load_old_config().
void Endstops::configure()
{
    const EndstopsGlobalConfigT &g = global_config();
    hysteresis_mm = g.hysteresis_mm;
    home_z_first = g.home_z_first;
    for (homing_info_t &h : homing_axis) {
        if (h.axis_index > Z_AXIS) continue;
        const EndstopAxisConfigT &ac = axis_config(h.axis_index);
        h.fast_rate = ac.fast_homing_rate;
        h.slow_rate = ac.slow_homing_rate;
        h.retract = ac.homing_retract_mm;
        h.homing_position = h.home_direction ? ac.min_pos : ac.max_pos;
        h.max_travel = ac.max_travel;
    }
}

enum { MIN_PIN, MAX_PIN };

#define STEPS_PER_MM(a) (THEROBOT.motor_steps_per_mm(a))



void Endstops::on_module_loaded()
{
    this->status = NOT_HOMING;

    // Do not do anything if not enabled or if no pins are defined
    if (!global_config().module_enable || !load_old_config()) return;

    GcodeDispatch::add_handler(this);
    Settings::add(settings_slot, &Endstops::report_settings, this);
    ADD_MCODE(m119, 119, BESIDE_JOB, Endstops::report_switches);
    ADD_MCODE(m206, 206, IMMEDIATE, Endstops::set_home_offset);
    ADD_MCODE(m306, 306, BARRIER, Endstops::set_home_offset_here);

	service_timer.start();
}

// Get config using old deprecated syntax Does not support ABC
bool Endstops::load_old_config()
{
    for (int i = X_AXIS; i <= Z_AXIS; ++i) { // X_AXIS to Z_AXIS
        const EndstopAxisConfigT &ac = axis_config(i);
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset = 0;
        hinfo.past_edge = 0;
        hinfo.homed = false;
        hinfo.axis = 'X'+i;
        hinfo.axis_index = i;
        hinfo.pin_info = nullptr;

        // True homes to min.
        hinfo.home_direction = ac.homing_direction != 1;

        // motor alarm info
        if (PinSpec::connected(ac.motor_alarm_pin)) {
        	motor_alarm_info_t *info = new motor_alarm_info_t;
        	info->pin.from_spec(ac.motor_alarm_pin)->as_input();
            info->axis = 'X' + i;
            info->axis_index = i;
            motor_alarms.push_back(info);
        }

        hinfo.motor_alarm_pin.from_spec(ac.motor_alarm_pin)->as_input();

        const uint16_t pins[2] = { ac.min_endstop, ac.max_endstop };
        for (int j = MIN_PIN; j <= MAX_PIN; ++j) {
            endstop_info_t *info = new endstop_info_t;
            info->pin.from_spec(pins[j])->as_input();
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
            info->axis = 'X' + i;
            info->axis_index = i;
            info->at_end = true;
            info->at_max = (j == MAX_PIN);

            // limits enabled
            info->limit_enable = ac.limit_enable;
        }

        homing_axis.push_back(hinfo);
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

    get_global_configs();
    configure();
    for(auto& a : motor_alarms) alarm_pins.add(a->pin);
    arm_limits();

    return true;
}

void Endstops::get_global_configs()
{
    const EndstopsGlobalConfigT &g = global_config();
	this->cover_endstop_pin.from_spec(g.cover_endstop)->as_input();

    // see if an order has been specified, must be three or more characters, XYZABC or ABYXZ etc
    string order = g.homing_order;

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

void Endstops::back_off_home(axis_bitmap_t axis)
{
    float delta[k_max_actuators]{0};
    bool moving= false;
    float slow_rate= NAN; // default mm/sec

    this->status = BACK_OFF_HOME;

    {
        for( auto& e : homing_axis) {
            if(!axis[e.axis_index]) continue; // only for axes we asked to move
            if(e.pin_info == nullptr) continue;
            // the position was set at the switch edge and the axis stands past_edge beyond
            // it; the retract is measured from the edge
            delta[e.axis_index]= e.retract * (e.home_direction ? 1 : -1) - e.past_edge;
            moving= true;
            // select slowest of them all
            slow_rate= std::isnan(slow_rate) ? e.slow_rate : std::min(slow_rate, e.slow_rate);
        }
    }

    if(moving) THEROBOT.delta_move_sync(delta, slow_rate, THEROBOT.get_number_registered_motors());

    this->status = NOT_HOMING;
}

// after homing X and Y the machine script may take over (after_home.ngc)
void Endstops::after_home(axis_bitmap_t axis)
{
    if(!axis[X_AXIS] || !axis[Y_AXIS]) return;
    scripts.run_sub("after_home", nullptr, 0);
}

// the switch an axis homes to is expected to be pressed for the whole cycle, including the
// retract off it; any other switch is a crash
void Endstops::service()
{
    check_motor_alarms();

    if(status == LIMIT_TRIGGERED) {
        for(auto& i : endstops) {
            if(i->limit_enable && i->pin.get()) { limit_clear_ms= 0; return; }
        }
        // every limit has to stay released, not just the one that tripped
        if(limit_clear_ms++ >= LIMIT_RELEASE_MS) {
            status= NOT_HOMING;
            THEKERNEL->step_ticker.clear_limit();
        }
        return;
    }

    if(!THEKERNEL->step_ticker.limit_hit()) return;
    status= LIMIT_TRIGGERED;
}

void Endstops::check_motor_alarms()
{
    if(machine_task.is_halted() || !alarm_pins.any()) return;
    for(auto& i : motor_alarms) {
        if(i->pin.get()) {
            char msg[32];
            snprintf(msg, sizeof(msg), "%c motor alarm", i->axis);
            machine_task.halt(MOTOR_ERROR_X + i->axis_index, msg);
            return;
        }
    }
}

void Endstops::arm_limits(const endstop_info_t *approaching)
{
    StepTicker::Limit l[k_max_actuators * 2];
    uint8_t n= 0;
    uint16_t steps= 0;
    for(auto& e : endstops) {
        if(!e->limit_enable || e == approaching) continue;
        if(n >= k_max_actuators * 2) break;
        l[n++]= StepTicker::Limit{e->pin, e->axis_index, e->at_end, e->at_max};
        uint16_t s= hysteresis_steps(e->axis_index);
        if(s > steps) steps= s;
    }
    THEKERNEL->step_ticker.set_limits(l, n, steps);
}

void Endstops::unhome_all()
{
    for (auto &p : homing_axis) p.homed= false;
}

uint16_t Endstops::hysteresis_steps(uint8_t axis) const
{
    return (uint16_t)(hysteresis_mm * THEROBOT.motor_steps_per_mm(axis));
}

bool Endstops::approach(uint8_t axis, float distance, float rate)
{
    homing_info_t &h= homing_axis[axis];
    if(h.pin_info == nullptr) return false;

    approach_watch.inputs.clear();
    approach_watch.witness.clear();
    approach_watch.inputs.add(h.pin_info->pin);
    approach_watch.motors= 1 << axis;
    // first contact is the hit: an axis that starts on its switch must not travel into the frame
    // first, and the retract and second touch measure anyway
    approach_watch.hysteresis= 0;

    arm_limits(h.pin_info);

    float delta[k_max_actuators]{0};
    delta[axis]= h.home_direction ? -distance : distance;

    bool ok= THEROBOT.delta_move_watch(delta, rate, homing_axis.size(), approach_watch);
    bool hit= approach_watch.hit;
    float steps_per_mm= THEROBOT.motor_steps_per_mm(axis);
    h.past_edge= hit && steps_per_mm != 0 ? (THEROBOT.motor_step(axis) - THEKERNEL->step_ticker.latch().steps[axis]) / steps_per_mm : 0;
    approach_watch.inputs.clear();
    arm_limits();
    return ok && hit;
}

// the fast pass finds the switch, the slow pass sets the position
bool Endstops::home_axis(uint8_t axis)
{
    homing_info_t &h= homing_axis[axis];

    if(!approach(axis, h.max_travel, h.fast_rate)) return false;

    float delta[k_max_actuators]{0};
    delta[axis]= h.home_direction ? h.retract : -h.retract;
    if(!THEROBOT.delta_move_sync(delta, h.slow_rate, homing_axis.size())) return false;

    return approach(axis, h.retract * 2, h.slow_rate);
}

void Endstops::home(axis_bitmap_t a)
{
    this->axis_to_home= a;
    this->status = HOMING;

    Robot::NoSegmentation no_segmentation;   // homing won't work with it enabled

    // Z first by default: the spindle has to be clear of the work before XY move
    uint8_t order[axis_bitmap_t().size()];
    uint8_t n= 0;
    if(home_z_first && axis_to_home[Z_AXIS]) order[n++]= Z_AXIS;
    if(axis_to_home[X_AXIS]) order[n++]= X_AXIS;
    if(axis_to_home[Y_AXIS]) order[n++]= Y_AXIS;
    if(!home_z_first && axis_to_home[Z_AXIS]) order[n++]= Z_AXIS;
    for (size_t i = A_AXIS; i < homing_axis.size(); ++i) if(axis_to_home[i]) order[n++]= i;

    for (uint8_t i = 0; i < n; ++i) {
        if(!home_axis(order[i])) {
            THEROBOT.reset_position_from_current_actuator_position();
            this->status = NOT_HOMING;
            if(!machine_task.is_halted()) machine_task.halt(HOME_FAIL, "homing failed");
            return;
        }
    }

    THEROBOT.reset_position_from_current_actuator_position();
    this->status = NOT_HOMING;
}

void Endstops::process_home_command(Gcode* gcode)
{
    bool axis_speced = (gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ||
                        gcode->has_letter('A') || gcode->has_letter('B') || gcode->has_letter('C'));

    axis_bitmap_t haxis;
    haxis.reset();
    for (auto &p : homing_axis) {
        // only enable homing if the endstop is defined,
        if(p.pin_info == nullptr) continue;
        if(!axis_speced || gcode->has_letter(p.axis)) haxis.set(p.axis_index);
    }

    home_axes(haxis);
}

void Endstops::home_all()
{
    axis_bitmap_t haxis;
    haxis.reset();
    for (auto &p : homing_axis) {
        if(p.pin_info != nullptr) haxis.set(p.axis_index);
    }
    home_axes(haxis);
}

void Endstops::home_axes(axis_bitmap_t haxis)
{
    if(haxis.none()) {
        printk("WARNING: Nothing to home\n");
        return;
    }

    THECONVEYOR.wait_for_idle();

    // turn off any compensation transform so Z does not move as XY home
    auto savect= THEROBOT.take_compensation();

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
            if(machine_task.is_halted()) break;
        }

    } else {
        // they could all home at the same time
        home(haxis);
    }

    THEROBOT.put_compensation(savect);

    if(machine_task.is_halted()) {
        for (auto &p : homing_axis) p.homed= false;
        return;
    }

    // Zero the ax(i/e)s position, add in the home offset
    // NOTE that if compensation is active the Z will be set based on where XY are, so make sure XY are homed first then Z
    // so XY are at a known consistent position.  (especially true if using a proximity probe)
    for (auto &p : homing_axis) {
        if (haxis[p.axis_index]) { // if we requested this axis to home
            // the switch closed where the position is defined; the axis stands a little past it
            THEROBOT.reset_axis_position(p.homing_position + p.home_offset + p.past_edge, p.axis_index);
            // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
            p.homed= true;
        }
    }

    // on some systems where 0,0 is bed center it is nice to have home goto 0,0 after homing
    // default is off for cartesian and on for deltas
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
            printk("error: Axis X must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[X_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('X')) - pos[X_AXIS]);
        homing_axis[X_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Y')) {
        if(!homing_axis[Y_AXIS].homed) {
            printk("error: Axis Y must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Y_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('Y')) - pos[Y_AXIS]);
        homing_axis[Y_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Z')) {
        if(!homing_axis[Z_AXIS].homed) {
            printk("error: Axis Z must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Z_AXIS].home_offset += (THEROBOT.to_millimeters(gcode->get_value('Z')) - pos[Z_AXIS]);
        homing_axis[Z_AXIS].homed= false; // force it to be homed
    }

    printk("Homing Offset: X %5.3f Y %5.3f Z %5.3f will take effect next home\n", homing_axis[X_AXIS].home_offset, homing_axis[Y_AXIS].home_offset, homing_axis[Z_AXIS].home_offset);
}


// parse gcodes
void Endstops::report_settings(void *self)
{
    Endstops *e= (Endstops *)self;
    printk(";Home offset (mm):\nM206 ");
    for (auto &p : e->homing_axis) {
        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
        printk("%c%1.2f ", p.axis, p.home_offset);
    }
    printk("\n");

    if(e->g28_position[X_AXIS] != 0 || e->g28_position[Y_AXIS] != 0) {
        printk(";predefined position:\nG28.1 X%1.4f Y%1.4f\n", e->g28_position[X_AXIS], e->g28_position[Y_AXIS]);
    }
}

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
                    printk("%c:%d ", p.axis, p.homed);
                }
                printk("\n");
                break;

            default:
                printk("error:Unsupported command\n");
                break;
        }

    }
}

// M119: every switch this machine has, homing ones by name and the rest by pin
void Endstops::report_switches(Gcode *gcode)
{
    for(auto& h : homing_axis) {
        if(h.pin_info == nullptr) continue; // ignore if not a homing endstop
        string name;
        name.append(1, h.axis).append(h.home_direction ? "_min" : "_max");
        printk("%s:%d ", name.c_str(), h.pin_info->pin.get());
    }
    printk("pins- ");
    for(auto& p : endstops) {
        string str(1, p->axis);
        if(p->limit_enable) str.append("L");
        printk("(%s)P%d.%d:%d ", str.c_str(), p->pin.port_number, p->pin.pin, p->pin.get());
    }
    printk(" Probe: %d", zprobe.getProbeStatus());
    printk("\n");
}

// M206: the offset applied at the next home
void Endstops::set_home_offset(Gcode *gcode)
{
    for (auto &p : homing_axis) {
        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
        if (gcode->has_letter(p.axis)) p.home_offset= gcode->get_value(p.axis);
    }

    for (auto &p : homing_axis) {
        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
        printk("%c: %5.3f ", p.axis, p.home_offset);
    }

    printk(" will take effect next home\n");
}

// M306: the same offset, from where the machine stands now
void Endstops::set_home_offset_here(Gcode *gcode)
{
    set_homing_offset(gcode);
}

bool Endstops::is_homing() const
{
    return status != NOT_HOMING;
}

// six limit-enabled endstops then the cover, as the status report expects them
void Endstops::get_endstop_states(char *data) const
{
    int index = 0;
    for(auto& i : endstops) {
        if(index < 6 && i->limit_enable) data[index++] = (char)i->pin.get();
    }
    data[5] = (char)this->cover_endstop_pin.get();
}

