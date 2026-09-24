/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KERNEL_H
#define KERNEL_H

#include "Module.h"
#include "StreamOutputPool.h"
#include "StepTicker.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "Adc.h"
#include "I2C.h" // mbed.h lib
#include <array>
#include <vector>
#include <string>

#include "FreeRTOS.h"
#include "timers.h"

// 9 WCS offsets
#define MAX_WCS 9UL
//Module manager
class Config;
class Module;
class Conveyor;
class SerialConsole;
class GcodeDispatch;
class Robot;
class SimpleShell;

enum STATE {
	IDLE    = 0,
	RUN     = 1,
	HOLD    = 2,
	HOME    = 3,
	ALARM   = 4,
	SLEEP   = 5,
	SUSPEND = 6,
	WAIT    = 7
};


class Kernel {
    public:
        Kernel() {};

        ~Kernel() {
            delete this->i2c;
        }

        void init();

        void printk(const char* format, ...) __attribute__ ((format(printf, 2, 3)));
        void vprintk(const char* format, va_list args);

        void add_module(Module* module);
        void register_for_event(_EVENT_ENUM id_event, Module *module);
        void call_event(_EVENT_ENUM id_event, void * argument= nullptr);

        bool kernel_has_event(_EVENT_ENUM id_event, Module *module);
        void unregister_for_event(_EVENT_ENUM id_event, Module *module);

        bool is_using_leds() const { return use_leds; }
        // safe from an interrupt
        void serve_main();

        void set_feed_hold(bool f) { feed_hold= f; }
        bool get_feed_hold() const { return feed_hold; }
        bool is_feed_hold_enabled() const { return enable_feed_hold; }
        void set_bad_mcu(bool b) { bad_mcu= b; }
        bool is_bad_mcu() const { return bad_mcu; }

        void set_laser_mode(bool f) { laser_mode = f; }
        bool get_laser_mode() const { return laser_mode; }

        void set_vacuum_mode(bool f) { vacuum_mode = f; }
        bool get_vacuum_mode() const { return vacuum_mode; }

        void set_optional_stop_mode(bool f) { optional_stop_mode = f; }
        bool get_optional_stop_mode() const { return optional_stop_mode; }

        void set_sleeping(bool f) { sleeping = f; }
        bool is_sleeping() const { return sleeping; }

        void set_suspending(bool f) { suspending = f; }
        bool is_suspending() const { return suspending; }

        void set_waiting(bool f) { waiting = f; }
        bool is_waiting() const { return waiting; }

        std::string get_query_string();

        std::string get_diagnose_string();

        // These modules are available to all other modules
        SerialConsole*    serial;
        StreamOutputPool  streams;
        Planner           planner;
        Config*           config;
        SlowTicker        slow_ticker;
        StepTicker        step_ticker;
        Adc               adc;
        uint32_t          base_stepping_frequency;

        uint8_t get_state();

    private:
        // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered
        mbed::I2C* i2c;
        std::array<std::vector<Module*>, NUMBER_OF_DEFINED_EVENTS> hooks;

        struct {
            bool use_leds:1;
            bool feed_hold:1;
            volatile bool enable_feed_hold:1;
            bool bad_mcu:1;
            bool laser_mode:1;
            bool vacuum_mode:1;
            bool optional_stop_mode:1;
            bool sleeping:1;
            bool suspending: 1;
            bool waiting: 1;
        };

};

extern Kernel* THEKERNEL;
extern Conveyor THECONVEYOR;
extern Robot THEROBOT;
extern SimpleShell simpleshell;
extern GcodeDispatch gcode_dispatch;    

#endif
