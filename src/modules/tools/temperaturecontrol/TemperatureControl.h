/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURECONTROL_H
#define TEMPERATURECONTROL_H

#include "Module.h"
class Gcode;
#include "Pwm.h"
#include "TempSensor.h"
#include "TemperatureControlPublicAccess.h"
#include "SoftTimer.h"

class TemperatureControl : public Module {

    public:
        uint16_t get_name() const { return name_checksum; }
        void get_status(struct pad_temperature *t);
        TemperatureControl(uint16_t name, int index)
        : thermistor_timer("ThermistorReading", 100, true, this, &TemperatureControl::thermistor_read_tick),
        name_checksum(name),
        pool_index(index),
        temp_violated(false),
        sensor(nullptr)
        {}

        ~TemperatureControl();

        void on_module_loaded();
        void on_main_loop(void* argument);
        void on_gcode_received(Gcode *argument);
        void on_second_tick(void* argument);

        float get_temperature();

    private:
        void load_config();
        void thermistor_read_tick();

        SoftTimer thermistor_timer;

        int pool_index;

        float target_temperature;
        float max_temp, min_temp;

        TempSensor *sensor;
        int o;
        float last_reading;
        float readings_per_second;

        std::string designator;

        // pack these to save memory
        struct {
            uint16_t name_checksum;
            uint16_t get_m_code:10;
            bool temp_violated:1;
            bool sensor_settings:1;
        };
};

#endif
