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

#define readings_per_second_checksum       CHECKSUM("readings_per_second")
#define max_temp_checksum                  CHECKSUM("max_temp")
#define min_temp_checksum                  CHECKSUM("min_temp")

#define get_m_code_checksum                CHECKSUM("get_m_code")

#define designator_checksum                CHECKSUM("designator")

TemperatureControl::~TemperatureControl()
{
    delete sensor;
}

void TemperatureControl::on_module_loaded()
{

    this->load_config();

    overheat_timer.start();

}

// Get configuration from the config file
void TemperatureControl::load_config()
{

    // General config
    this->get_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();
    float readings_per_second = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(20)->as_number();

    this->designator          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Max and min temperatures we are not allowed to get over (Safety)
    this->max_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, max_temp_checksum)->by_default(300)->as_number();
    this->min_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, min_temp_checksum)->by_default(0)->as_number();

    delete sensor;
    sensor = new Thermistor();
    sensor->UpdateConfig(temperature_control_checksum, this->name_checksum);

    // reading tick
    thermistor_timer.setFrequency(readings_per_second);
    thermistor_timer.start();

    this->last_reading = 0.0;
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

void TemperatureControl::thermistor_read_tick()
{
    last_reading = sensor->get_temperature();
}

// its own timer, not the second tick: a check that stops the spindle must not depend on some
// task getting round to pumping an event
void TemperatureControl::overheat_tick()
{
    if(THEKERNEL->is_halted()) return;

    // read here and not from last_reading: a stalled reading timer would hide the overheat
    float t = sensor->get_temperature();
    bool sane = isfinite(t);
    if(sane && t >= min_temp && t <= max_temp) return;

    // whole degrees: the halt message is printed from the main loop and floats are dear here
    char msg[32];
    if(sane) snprintf(msg, sizeof(msg), "%s at %dC, max %d", designator.c_str(), (int)t, (int)max_temp);
    else snprintf(msg, sizeof(msg), "%s sensor open", designator.c_str());
    THEKERNEL->halt(SPINDLE_OVERHEATED, msg);
}

