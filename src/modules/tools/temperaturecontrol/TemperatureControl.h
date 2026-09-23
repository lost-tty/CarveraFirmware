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
        : read_timer("SpindleTemp", 1000, true, this, &TemperatureControl::read_tick),
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
        void read_tick();
        void drive_fan(float temp);

        SoftTimer read_timer;

        int pool_index;

        float max_temp, min_temp;

        TempSensor *sensor;
        float last_reading;
        bool has_reading;

        static const uint8_t k_settle_ticks= 4;   // ticks a missing reading is tolerated at startup
        uint8_t bad_readings;

        // the fan follows a curve rather than a setpoint: this is a limit, not a temperature
        // the spindle is meant to hold
        float fan_threshold, fan_power_init, fan_power_step, fan_power_laser;
        uint16_t fan_cooldown_delay, cooling_since;
        uint16_t fan_switch_cs;

        std::string designator;

        uint16_t name_checksum;
        uint16_t get_m_code;
};

#endif
