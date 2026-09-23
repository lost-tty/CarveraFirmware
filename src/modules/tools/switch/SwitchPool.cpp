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
#include "SwitchPool.h"
#include "Switch.h"
#include "SwitchPublicAccess.h"
#include "Switch.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"

#define switch_checksum CHECKSUM("switch")
#define enable_checksum CHECKSUM("enable")

std::vector<Switch *> SwitchPool::switches;

Switch *SwitchPool::find(uint16_t name)
{
    for(Switch *s : switches) {
        if(s->get_name() == name) return s;
    }
    return nullptr;
}

bool SwitchPool::get_state(uint16_t name, struct pad_switch *pad)
{
    Switch *s = find(name);
    if(s == nullptr) return false;
    s->get_state(pad);
    return true;
}

bool SwitchPool::set_state(uint16_t name, bool on)
{
    Switch *s = find(name);
    if(s == nullptr) return false;
    s->set_state(on);
    return true;
}

bool SwitchPool::set_state(uint16_t name, bool on, float value)
{
    Switch *s = find(name);
    if(s == nullptr) return false;
    s->set_state(on, value);
    return true;
}

// the same code may be one switch's on command and another's off command
void SwitchPool::run_switch_gcode(Gcode *gcode)
{
    for(Switch *s : switches) {
        if(s->get_subcode() != gcode->subcode) continue;
        if(s->get_on_mcode() == gcode->m) s->on_gcode(gcode);
        else if(s->get_off_mcode() == gcode->m) s->off_gcode(gcode);
    }
}

void SwitchPool::claim(uint16_t code, uint8_t subcode)
{
    if(code == 0) return;
    for(size_t i = 0; i < used; i++) {
        if(codes[i].number == code && codes[i].subcode == subcode) return;
    }
    // a refused slot was never written, so it must not count against the budget
    if(ADD_SUBCODE(codes[used], code, subcode, ACTION, SwitchPool::run_switch_gcode)) used++;
}

void SwitchPool::load_tools()
{
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, switch_checksum );

    for( unsigned int i = 0; i < modules.size(); i++ ) {
        // If module is enabled
        if( THEKERNEL->config->value(switch_checksum, modules[i], enable_checksum )->as_bool() == true ) {
            Switch *controller = new Switch(modules[i]);
            switches.push_back(controller);
            THEKERNEL->add_module(controller);
        }
    }

    // the registry holds the address of each slot, so the vector must never grow again
    if(!codes.empty()) return;
    codes.resize(switches.size() * 2);
    for(Switch *s : switches) {
        claim(s->get_on_mcode(), s->get_subcode());
        claim(s->get_off_mcode(), s->get_subcode());
    }
}





