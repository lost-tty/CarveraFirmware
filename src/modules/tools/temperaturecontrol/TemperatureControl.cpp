/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "GcodeDispatch.h"
#include <math.h>
#include "TemperatureControl.h"
#include "TemperatureControlPool.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"

#include "Logging.h"
#include "Config.h"
#include "checksumm.h"
#include "Gcode.h"
#include "ConfigValue.h"
#include "SerialMessage.h"
#include "utils.h"
#include "StreamOutput.h"

// Temp sensor implementations:
#include "Thermistor.h"

#include "MRI_Hooks.h"

#define UNDEFINED -1

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

    // We start not desiring any temp
    this->target_temperature = UNDEFINED;
    this->sensor_settings= false; // set to true if sensor settings have been overriden

    // Settings
    this->load_config();

    // Register for events
    GcodeDispatch::add_handler(this);
    this->register_for_event(ON_SECOND_TICK);

}

void TemperatureControl::on_main_loop(void *argument)
{
	if(THEKERNEL->is_halted()) return;
    if (this->temp_violated) {
        this->temp_violated = false;
        printk("ERROR: Spindle overheated, max - %f°C, current - %f°C !\n", max_temp, get_temperature());
        THEKERNEL->halt(SPINDLE_OVERHEATED);
    }
}

// Get configuration from the config file
void TemperatureControl::load_config()
{

    // General config
    this->get_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();
    this->readings_per_second = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(20)->as_number();

    this->designator          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Max and min temperatures we are not allowed to get over (Safety)
    this->max_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, max_temp_checksum)->by_default(300)->as_number();
    this->min_temp = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, min_temp_checksum)->by_default(0)->as_number();

    delete sensor;
    sensor = new Thermistor();
    sensor->UpdateConfig(temperature_control_checksum, this->name_checksum);

    // sigma-delta output modulation
    this->o = 0;

    // reading tick
    thermistor_timer.setFrequency(this->readings_per_second);
    thermistor_timer.start();

    this->last_reading = 0.0;
}

void TemperatureControl::on_gcode_received(Gcode *argument)
{
    Gcode *gcode = argument;
    if (gcode->has_m) {

        if( gcode->m == this->get_m_code ) {
            char buf[32]; // should be big enough for any status
            int n = snprintf(buf, sizeof(buf), "%s:%3.1f /%3.1f @%d ", this->designator.c_str(), this->get_temperature(), ((target_temperature <= 0) ? 0.0 : target_temperature), this->o);
            gcode->txt_after_ok.append(buf, n);
            return;
        }

        if (gcode->m == 305) { // set or get sensor settings
            if (gcode->has_letter('S') && (gcode->get_value('S') == this->pool_index)) {
                TempSensor::sensor_options_t args= gcode->get_args();
                args.erase('S'); // don't include the S
                if(args.size() > 0) {
                    // set the new options
                    if(sensor->set_optional(args)) {
                        this->sensor_settings= true;
                    }else{
                        gcode->stream->printf("Unable to properly set sensor settings, make sure you specify all required values\n");
                    }
                }else{
                    // don't override
                    this->sensor_settings= false;
                }

            }else if(!gcode->has_letter('S')) {
                sensor->get_raw();
                TempSensor::sensor_options_t options;
                if(sensor->get_optional(options)) {
                    for(auto &i : options) {
                        // foreach optional value
                        gcode->stream->printf("%s(S%d): %c %1.18f\n", this->designator.c_str(), this->pool_index, i.first, i.second);
                    }
                }
            }

            return;
        }

    }
}

void TemperatureControl::get_status(struct pad_temperature *t)
{
    t->current_temperature = this->get_temperature();
    t->target_temperature = (target_temperature <= 0) ? 0 : this->target_temperature;
    t->pwm = this->o;
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

/**
 * Based on https://github.com/br3ttb/Arduino-PID-Library
 */

void TemperatureControl::on_second_tick(void *argument)
{
    if(THEKERNEL->is_halted()) return;

    float temperature = sensor->get_temperature();
    if (isinf(temperature) || temperature < min_temp || temperature > max_temp) {
        printk("ERROR: Spindle overheated, max - %1.1f, current - %1.1f\n", max_temp, temperature);
        THEKERNEL->halt(SPINDLE_OVERHEATED);
    }
}

