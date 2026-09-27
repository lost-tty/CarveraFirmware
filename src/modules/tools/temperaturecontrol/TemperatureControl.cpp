/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include <math.h>
#include "TemperatureControl.h"
#include "TemperatureControlPool.h"
#include "SpindleTempConfig.h"

#include "Logging.h"
#include "Gcode.h"
#include "utils.h"
#include "StreamOutput.h"

// Temp sensor implementations:
#include "Thermistor.h"
#include "SwitchPool.h"
#include "modules/robot/MachineTask.h"

TemperatureControl::~TemperatureControl()
{
    delete sensor;
}

uint16_t TemperatureControl::get_report_mcode() const { return cfg.report_mcode; }

// The sensor is set up once in load_config(); only these settings change at runtime.
void TemperatureControl::configure()
{
    const SpindleTempConfigT &t = spindle_temp_cfg();
    const TempSwitchConfigT &s = temp_switch_cfg();
    cfg.min_temp = t.min_temp;
    cfg.max_temp = t.max_temp;
    strncpy(cfg.designator, t.designator, sizeof(cfg.designator));
    cfg.threshold_temp = s.threshold_temp;
    cfg.power_init = s.cooldown_power_init;
    cfg.power_step = s.cooldown_power_step;
    cfg.power_laser = s.cooldown_power_laser;
    cfg.cooldown_delay = s.cooldown_delay;
    uint8_t fan= s.fan_switch;
    fan_switch_cs= fan == ConfigTable::ENUM_INVALID ? 0 : get_checksum(switch_names[fan]);
}

void TemperatureControl::on_module_loaded()
{

    this->load_config();

    read_timer.start();

}

// Get configuration from the config file
void TemperatureControl::load_config()
{

    delete sensor;
    sensor = new Thermistor();
    sensor->UpdateConfig();

    // The pool registers the report M-code once, so it is read only at boot.
    cfg.report_mcode = spindle_temp_cfg().get_m_code;
    configure();
    cooling_since= cfg.cooldown_delay + 1;   // starts off

    has_reading= false;
    bad_readings= 0;
    last_reading = 0.0;
}

void TemperatureControl::report_temperature(Gcode *gcode)
{
    gcode->stream->printf("%s:%3.1f /0.0 @0\n", cfg.designator, this->get_temperature());
}

// the pool has already checked that S names this controller, or that there is no S at all
void TemperatureControl::sensor_settings_gcode(Gcode *gcode)
{
    if(gcode->has_letter('S')) {
        TempSensor::sensor_options_t args= gcode->get_args();
        args.erase('S'); // don't include the S
        if(args.size() > 0 && !sensor->set_optional(args)) {
            gcode->stream->printf("Unable to properly set sensor settings, make sure you specify all required values\n");
        }
    } else {
        sensor->get_raw();
        TempSensor::sensor_options_t options;
        if(sensor->get_optional(options)) {
            for(auto &i : options) {
                gcode->stream->printf("%s(S%d): %c %1.18f\n", cfg.designator, this->pool_index, i.first, i.second);
            }
        }
    }
}

void TemperatureControl::get_status(struct pad_temperature *t)
{
    t->current_temperature = this->get_temperature();
    t->target_temperature = 0;
    t->pwm = 0;
    t->designator = cfg.designator;
    t->id = this->name_checksum;
}

float TemperatureControl::get_temperature()
{
    return last_reading;
}

// One timer: the overheat check comes first, then the fan curve. The ADC already averages,
// so this works on the reading as it comes.
void TemperatureControl::read_tick()
{
    float t= sensor->get_temperature();

    // the ADC says nothing until it has averaged its first samples, which looks the same as an
    // open sensor: give it a few ticks to come up, then a reading that is still missing is a fault
    if(!isfinite(t)) {
        if(!machine_task.is_halted() && ++bad_readings > k_settle_ticks) {
            char msg[32];
            snprintf(msg, sizeof(msg), "%s sensor open", cfg.designator);
            machine_task.halt(SPINDLE_OVERHEATED, msg);
        }
        return;
    }
    bad_readings= 0;

    last_reading= t;
    has_reading= true;

    if(!machine_task.is_halted() && (t < cfg.min_temp || t > cfg.max_temp)) {
        char msg[32];
        snprintf(msg, sizeof(msg), "%s at %dC, max %d", cfg.designator, (int)t, (int)cfg.max_temp);
        machine_task.halt(SPINDLE_OVERHEATED, msg);
        return;
    }

    drive_fan(t);
}

// The fan is a limiter, not a controller: off below the threshold, rising with the temperature
// above it, and running on for cooldown_delay seconds after the spindle goes cool.
void TemperatureControl::drive_fan(float temp)
{
    if(fan_switch_cs == 0) return;

    float power= 0;
    if(THEKERNEL->get_laser_mode()) power= cfg.power_laser;
    else if(temp >= cfg.threshold_temp) {
        power= cfg.power_init
            + (temp - cfg.threshold_temp) * cfg.power_step;
    }

    if(power > 0) {
        cooling_since= 0;
        SwitchPool::set_state(fan_switch_cs, true, power);
        return;
    }

    if(cooling_since > cfg.cooldown_delay) return;   // already off
    if(++cooling_since > cfg.cooldown_delay) SwitchPool::set_state(fan_switch_cs, false);
}
