/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "Pin.h"
#include "SoftTimer.h"
#include "libs/PinGroup.h"
#include "libs/Watch.h"

#include <bitset>
#include <array>
#include <map>

class StepperMotor;
class Gcode;
class Pin;

class Endstops : public Module{

    public:
        bool is_homing() const;
        bool is_homed(uint8_t axis) const { return homing_axis[axis].homed; }
        bool cover_closed() const { return cover_endstop_pin.get(); }
        void get_endstop_states(char *data) const;
        const float *get_g28_position() const { return g28_position; }
        Endstops()
        : service_timer("Endstops", 1, true, this, &Endstops::service)
        {}

        void on_module_loaded();
        void on_gcode_received(Gcode *argument);

    private:
        bool load_old_config();
        bool load_config();
        void get_global_configs();
        using axis_bitmap_t = std::bitset<6>;
        void home(axis_bitmap_t a);
        void back_off_home(axis_bitmap_t axis);
        void after_home(axis_bitmap_t axis);
        void process_home_command(Gcode* gcode);
        void set_homing_offset(Gcode* gcode);
        void service();
        void check_motor_alarms();
        bool approach(uint8_t axis, float distance, float rate);
        bool home_axis(uint8_t axis);
        uint16_t hysteresis_steps(uint8_t axis) const;

        SoftTimer service_timer;

        // global settings
        float g28_position[3]{0}; // save G28 (in grbl mode)
        float     hysteresis_mm;
        uint32_t  limit_clear_ms{0};
        static const uint32_t LIMIT_RELEASE_MS = 100;
        axis_bitmap_t axis_to_home;


        Pin cover_endstop_pin;

        // per endstop settings
        using endstop_info_t = struct {
            Pin pin;
            char     axis{0};          // one of XYZABC
            uint8_t  axis_index{0};
            bool     limit_enable{false};
            bool     at_end{false};
            bool     at_max{false};
        };

        // motor alarm settings
        using motor_alarm_info_t = struct {
        	Pin pin;
            struct {
                char axis:8; // one of XYZABC
                uint8_t axis_index:3;
            };
        };


        void arm_limits(const endstop_info_t *approaching = nullptr);

        using homing_info_t = struct {
            float homing_position;
            float home_offset;
            float max_travel;
            float retract;
            float fast_rate;
            float slow_rate;
            float past_edge;
            endstop_info_t *pin_info;
            Pin motor_alarm_pin;

            struct {
                char axis:8; // one of XYZABC
                uint8_t axis_index:3;
                bool home_direction:1; // true min or false max
                bool homed:1;
            };
        };

        // array of endstops
        std::vector<endstop_info_t *> endstops;

        // array of motor alarm
        std::vector<motor_alarm_info_t *> motor_alarms;

        // axis that can be homed, 0,1,2 always there and optionally 3 is A, 4 is B, 5 is C
        std::vector<homing_info_t> homing_axis;

        // its own byte: the service writes it from the timer task while homing drives the rest
        enum Status : char { NOT_HOMING, HOMING, BACK_OFF_HOME, LIMIT_TRIGGERED };
        volatile Status status{NOT_HOMING};
        PinGroup alarm_pins;
        Watch    approach_watch;   // the isr holds a pointer to this, so it outlives the move

        // Global state
        struct {
            uint32_t homing_order:18;
            bool home_z_first:1;
        };
};

extern Endstops endstops;
