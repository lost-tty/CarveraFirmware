/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
using namespace std;
#include <vector>
#include "TemperatureControlPool.h"
#include "TemperatureControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "TemperatureControlPublicAccess.h"
#include "Gcode.h"

#define enable_checksum              CHECKSUM("enable")

std::vector<TemperatureControl *> TemperatureControlPool::controls;

TemperatureControl *TemperatureControlPool::find(uint16_t name)
{
    for(TemperatureControl *c : controls) {
        if(c->get_name() == name) return c;
    }
    return nullptr;
}

bool TemperatureControlPool::get_temperature(uint16_t name, struct pad_temperature *t)
{
    TemperatureControl *c = find(name);
    if(c == nullptr) return false;
    c->get_status(t);
    return true;
}

void TemperatureControlPool::poll(std::vector<struct pad_temperature> &v)
{
    for(TemperatureControl *c : controls) {
        struct pad_temperature t;
        c->get_status(&t);
        v.push_back(t);
    }
}

// every controller that asked for this code answers on one line
void TemperatureControlPool::report_temperature(Gcode *gcode)
{
    for(TemperatureControl *c : controls) {
        if(c->get_report_mcode() == gcode->m) c->report_temperature(gcode);
    }
}

// M305 S<n> addresses one controller; without an S every controller reports
void TemperatureControlPool::sensor_settings_gcode(Gcode *gcode)
{
    for(TemperatureControl *c : controls) {
        if(gcode->has_letter('S') && gcode->get_value('S') != c->get_pool_index()) continue;
        c->sensor_settings_gcode(gcode);
    }
}

void TemperatureControlPool::claim(uint16_t code)
{
    for(size_t i = 0; i < used; i++) {
        if(report_codes[i].number == code) return;
    }
    // a refused slot was never written, so it must not count against the budget
    if(ADD_MCODE(report_codes[used], code, BESIDE_JOB, TemperatureControlPool::report_temperature)) used++;
}

void TemperatureControlPool::load_tools()
{
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, temperature_control_checksum );
    int cnt = 0;
    for( auto cs : modules ) {
        // If module is enabled
        if( THEKERNEL->config->value(temperature_control_checksum, cs, enable_checksum )->as_bool() ) {
            TemperatureControl *controller = new TemperatureControl(cs, cnt++);
            controls.push_back(controller);
            THEKERNEL->add_module(controller);
        }
    }

    if(controls.empty()) return;

    // the registry holds the address of each slot, so the vector must never grow again
    if(!report_codes.empty()) return;
    report_codes.resize(controls.size());
    for(TemperatureControl *c : controls) claim(c->get_report_mcode());

    ADD_MCODE(m305, 305, IMMEDIATE, TemperatureControlPool::sensor_settings_gcode);
}
