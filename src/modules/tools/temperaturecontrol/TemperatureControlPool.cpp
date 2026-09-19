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
}
