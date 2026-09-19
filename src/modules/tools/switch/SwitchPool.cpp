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

}





