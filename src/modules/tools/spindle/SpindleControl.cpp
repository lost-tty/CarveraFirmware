/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "SpindleControl.h"
#include "libs/Logging.h"
#include "SwitchPublicAccess.h"
#include "SwitchPool.h"
#include "ATCHandlerPublicAccess.h"
#include "ATCHandler.h"

SpindleControl *spindle_control = nullptr;

void SpindleControl::register_mcodes()
{
    ADD_MCODE(m223, 223, BESIDE_JOB, SpindleControl::handle_override);
    ADD_MCODE(m957, 957, IMMEDIATE, SpindleControl::handle_report);
    ADD_MCODE(m958, 958, BARRIER, SpindleControl::handle_pid);
}

void SpindleControl::start(Gcode *gcode)
{
    struct tool_status tool;
    bool tool_ok = atc_handler.get_tool_status(&tool) && tool.active_tool > 0;
    if(!tool_ok) {
        THEKERNEL->halt(MANUAL, "no tool set");
        printk("ERROR: No tool or probe tool!\n");
        return;
    }

    if(THEKERNEL->get_vacuum_mode()) {
        bool b = true;
        SwitchPool::set_state(vacuum_checksum, b);
    }

    if(gcode->has_letter('S')) set_speed(gcode->get_value('S'));
    if(!spindle_on) turn_on();
}

void SpindleControl::stop(Gcode *gcode)
{
    if(THEKERNEL->get_vacuum_mode()) {
        bool b = false;
        SwitchPool::set_state(vacuum_checksum, b);
    }

    if(spindle_on) turn_off();
}

void SpindleControl::handle_override(Gcode *gcode)
{
    if(!gcode->has_letter('S')) return;
    float factor = gcode->get_value('S');
    if(factor < 50.0F) factor = 50.0F;
    if(factor > 200.0F) factor = 200.0F;
    set_factor(factor);
}

void SpindleControl::handle_report(Gcode *gcode)
{
    report_speed();
}

void SpindleControl::handle_pid(Gcode *gcode)
{
    if(gcode->has_letter('P')) set_p_term(gcode->get_value('P'));
    if(gcode->has_letter('I')) set_i_term(gcode->get_value('I'));
    if(gcode->has_letter('D')) set_d_term(gcode->get_value('D'));
    report_settings();
}
