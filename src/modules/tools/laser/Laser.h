/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "SimpleShell.h"
class Gcode;

#include <string>

class StreamOutput;

#include "libs/Module.h"
#include "libs/Killable.h"
#include "GcodeDispatch.h"
#include "SoftTimer.h"

#include <stdint.h>

namespace mbed {
    class PwmOut;
}
class Pin;
class Block;

class Laser : public Module, public Killable {

    public:
        Laser()
        : laser_power_timer("LaserPower", 1, true, this, &Laser::set_proportional_power)
        {}

        void on_module_loaded();
        void enter_laser_mode(Gcode *);
        void enter_cnc_mode(Gcode *);
        void test_mode_on(Gcode *);
        void test_mode_off(Gcode *);
        void set_scale(Gcode *);

        GcodeDispatch::Mcode m321, m322, m323, m324, m325;
        void start(Gcode *gcode);
        void stop(Gcode *gcode);
        static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
        static const SimpleShell::Sub<Laser> SUBS[];
        void sub_on(std::string args, StreamOutput *stream);
        void sub_off(std::string args, StreamOutput *stream);
        void sub_status(std::string args, StreamOutput *stream);
        void sub_test(std::string args, StreamOutput *stream);
        SimpleShell::Registered shell_slot;

        void kill() override;
        void cleanup() override;
        void get_status(struct laser_status *t);
        void set_scale(float s) { scale= s/100; }
        float get_scale() const { return scale*100; }
        bool set_laser_power(float p);

    private:
        void set_proportional_power();
        bool get_laser_power(float& power) const;
        float current_speed_ratio(const Block *block) const;

        SoftTimer laser_power_timer;

        Pin *laser_pin= nullptr;
        mbed::PwmOut *pwm_pin= nullptr;
        Pin *ttl_pin= nullptr;
        float laser_test_power;    // laser power when doing calibration
        float laser_maximum_power; // maximum allowed laser power to be output on the pwm pin
        float laser_minimum_power; // value used to tickle the laser on moves.  Also minimum value for auto-scaling
        float laser_maximum_s_value; // Value of S code that will represent max power
        float scale;

        int32_t ms_per_tick; // ms between each ticks, depends on PWM frequency

        struct {
            bool laser_on:1;      // set if the laser is on
            bool pwm_inverting:1; // stores whether the PWM period should be inverted
            bool ttl_used:1;        // stores whether we have a TTL output
            bool ttl_inverting:1;   // stores whether the TTL output should be inverted
            bool testing:1;     // set when manually firing
        };
};

extern Laser laser;
