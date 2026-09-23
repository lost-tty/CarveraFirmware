/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPINDLE_CONTROL_MODULE_H
#define SPINDLE_CONTROL_MODULE_H

#include "libs/Module.h"
#include "libs/Killable.h"
#include "GcodeDispatch.h"
class Gcode;

struct spindle_status;

class SpindleControl: public Module, public Killable {
    public:
        SpindleControl() {};
        void start(Gcode *gcode);
        void stop(Gcode *gcode);
        virtual void get_status(struct spindle_status *t) {};
        void kill() override = 0;
        virtual ~SpindleControl() {};
        virtual void on_module_loaded() {};
        void register_mcodes();

    protected:
        bool spindle_on;

    private:
        void handle_override(Gcode *);
        void handle_report(Gcode *);
        void handle_pid(Gcode *);

        GcodeDispatch::Mcode m223, m957, m958;
        
        virtual void turn_on(void) {};
        virtual void turn_off(void) {};
        virtual void set_speed(int) {};
        virtual void report_speed(void) {};
        virtual void set_p_term(float) {};
        virtual void set_i_term(float) {};
        virtual void set_d_term(float) {};
        virtual void report_settings(void) {};

        virtual void set_factor(float) {};
};

#endif

// the one spindle SpindleMaker built, or nullptr if none is configured
extern SpindleControl *spindle_control;
