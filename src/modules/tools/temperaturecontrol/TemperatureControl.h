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
#include "TempSensor.h"
#include "TemperatureControlPublicAccess.h"
#include "SoftTimer.h"


class TemperatureControl : public Module {

    public:
        uint16_t get_name() const { return name_checksum; }
        uint16_t get_report_mcode() const { return get_m_code; }
        int get_pool_index() const { return pool_index; }
        void get_status(struct pad_temperature *t);
        TemperatureControl(uint16_t name, int index)
        : thermistor_timer("ThermistorReading", 100, true, this, &TemperatureControl::thermistor_read_tick),
        overheat_timer("OverheatCheck", 1000, true, this, &TemperatureControl::overheat_tick),
        name_checksum(name),
        pool_index(index),
        sensor(nullptr)
        {}

        ~TemperatureControl();

        void on_module_loaded();
        void report_temperature(Gcode *);
        void sensor_settings_gcode(Gcode *);

        float get_temperature();

    private:
        void load_config();
        void thermistor_read_tick();

        void overheat_tick();

        SoftTimer thermistor_timer;
        SoftTimer overheat_timer;

        int pool_index;

        float max_temp, min_temp;

        TempSensor *sensor;
        float last_reading;

        std::string designator;

        uint16_t name_checksum;
        uint16_t get_m_code;
};

#endif
