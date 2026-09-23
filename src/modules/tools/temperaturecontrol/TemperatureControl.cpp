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

#include "Logging.h"
#include "Config.h"
#include "checksumm.h"
#include "Gcode.h"
#include "ConfigValue.h"
#include "utils.h"
#include "StreamOutput.h"

// Temp sensor implementations:
#include "Thermistor.h"
#include "SwitchPool.h"

#define max_temp_checksum                  CHECKSUM("max_temp")
#define min_temp_checksum                  CHECKSUM("min_temp")

#define get_m_code_checksum                CHECKSUM("get_m_code")

#define designator_checksum                CHECKSUM("designator")

#define temperatureswitch_checksum         CHECKSUM("temperatureswitch")
#define switch_checksum                    CHECKSUM("switch")
#define threshold_temp_checksum            CHECKSUM("threshold_temp")
#define cooldown_power_init_checksum       CHECKSUM("cooldown_power_init")
#define cooldown_power_step_checksum       CHECKSUM("cooldown_power_step")
#define cooldown_power_laser_checksum      CHECKSUM("cooldown_power_laser")
#define cooldown_delay_checksum            CHECKSUM("cooldown_delay")

TemperatureControl::~TemperatureControl()
{
    delete sensor;
}

void TemperatureControl::on_module_loaded()
{

    this->load_config();

    read_timer.start();

}

// Get configuration from the config file
void TemperatureControl::load_config()
{

    // General config
    this->get_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();

    this->designator          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Max and min temperatures we are not allowed to get over (Safety)
    this->max_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, max_temp_checksum)->by_default(300)->as_number();
    this->min_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, min_temp_checksum)->by_default(0)->as_number();

    delete sensor;
    sensor = new Thermistor();
    sensor->UpdateConfig(temperature_control_checksum, this->name_checksum);

    // the fan curve; the keys keep their old temperatureswitch.<name>. spelling
    fan_threshold    = THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, threshold_temp_checksum)->by_default(35.0F)->as_number();
    fan_power_init   = THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, cooldown_power_init_checksum)->by_default(50.0F)->as_number();
    fan_power_step   = THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, cooldown_power_step_checksum)->by_default(10.0F)->as_number();
    fan_power_laser  = THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, cooldown_power_laser_checksum)->by_default(80.0F)->as_number();
    fan_cooldown_delay = THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, cooldown_delay_checksum)->by_default(180)->as_number();

    std::string fan= THEKERNEL->config->value(temperatureswitch_checksum, name_checksum, switch_checksum)->by_default("")->as_string();
    fan_switch_cs= fan.empty() ? 0 : get_checksum(fan);
    cooling_since= fan_cooldown_delay + 1;   // starts off

    has_reading= false;
    bad_readings= 0;
    last_reading = 0.0;
}

void TemperatureControl::report_temperature(Gcode *gcode)
{
    char buf[32]; // should be big enough for any status
    int n = snprintf(buf, sizeof(buf), "%s:%3.1f /0.0 @0 ", this->designator.c_str(), this->get_temperature());
    gcode->txt_after_ok.append(buf, n);
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
                gcode->stream->printf("%s(S%d): %c %1.18f\n", this->designator.c_str(), this->pool_index, i.first, i.second);
            }
        }
    }
}

void TemperatureControl::get_status(struct pad_temperature *t)
{
    t->current_temperature = this->get_temperature();
    t->target_temperature = 0;
    t->pwm = 0;
    t->designator = this->designator;
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
        if(!THEKERNEL->is_halted() && ++bad_readings > k_settle_ticks) {
            char msg[32];
            snprintf(msg, sizeof(msg), "%s sensor open", designator.c_str());
            THEKERNEL->halt(SPINDLE_OVERHEATED, msg);
        }
        return;
    }
    bad_readings= 0;

    last_reading= t;
    has_reading= true;

    if(!THEKERNEL->is_halted() && (t < min_temp || t > max_temp)) {
        char msg[32];
        snprintf(msg, sizeof(msg), "%s at %dC, max %d", designator.c_str(), (int)t, (int)max_temp);
        THEKERNEL->halt(SPINDLE_OVERHEATED, msg);
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
    if(THEKERNEL->get_laser_mode()) power= fan_power_laser;
    else if(temp >= fan_threshold) power= fan_power_init + (temp - fan_threshold) * fan_power_step;

    if(power > 0) {
        cooling_since= 0;
        SwitchPool::set_state(fan_switch_cs, true, power);
        return;
    }

    if(cooling_since > fan_cooldown_delay) return;   // already off
    if(++cooling_since > fan_cooldown_delay) SwitchPool::set_state(fan_switch_cs, false);
}
