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
#include "SwitchConfig.h"
#include "checksumm.h"
#include "Gcode.h"
#include "utils.h"

CONFIG_KEYS(switch_config_keys, SwitchConfigT, SWITCH_CONFIG);

static const int switch_count = sizeof(switch_names) / sizeof(switch_names[0]) - 1;

#define CMDS(on, off, pin) \
    CFG_SET(SwitchConfigT, input_on_command, on), \
    CFG_SET(SwitchConfigT, input_off_command, off), \
    CFG_SET(SwitchConfigT, output_pin, pin)
static const ConfigTable::Override vacuum_ov[] = {
    CMDS("M801", "M802", "2.13"),
    CFG_SET(SwitchConfigT, output_type, "digitalpwm"),
    CFG_SET(SwitchConfigT, pwm_pin, "2.3"),
    CFG_SET(SwitchConfigT, default_on_value, 80.0f),
};
static const ConfigTable::Override air_ov[] = { CMDS("M7", "M9", "0.11") };
static const ConfigTable::Override light_ov[] = {
    CMDS("M821", "M822", "2.0"),
    CFG_SET(SwitchConfigT, startup_state, true),
    CFG_SET(SwitchConfigT, ignore_on_halt, true),
};
static const ConfigTable::Override spindlefan_ov[] = {
    CMDS("M811", "M812", "2.1"),
    CFG_SET(SwitchConfigT, output_type, "hwpwm"),
    CFG_SET(SwitchConfigT, default_on_value, 50.0f),
};
static const ConfigTable::Override extend_ov[] = {
    CMDS("M851", "M852", "2.2"),
    CFG_SET(SwitchConfigT, output_type, "hwpwm"),
};
static const ConfigTable::Override toolsensor_ov[] = { CMDS("M831", "M832", "1.22") };
static const ConfigTable::Override probecharger_ov[] = {
    CMDS("M841", "M842", "0.23"),
    CFG_SET(SwitchConfigT, startup_state, true),
    CFG_SET(SwitchConfigT, ignore_on_halt, true),
};
#undef CMDS

static void switch_config_changed(const ConfigTable::Group *g, const void *cfg);
CONFIG_GROUPS(switch_pool_config_groups,
    CFG_GROUP_OV("switch.vacuum", switch_config_keys, SwitchConfigT, vacuum_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.air", switch_config_keys, SwitchConfigT, air_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.light", switch_config_keys, SwitchConfigT, light_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.spindlefan", switch_config_keys, SwitchConfigT, spindlefan_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.extend", switch_config_keys, SwitchConfigT, extend_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.toolsensor", switch_config_keys, SwitchConfigT, toolsensor_ov,
                 switch_config_changed),
    CFG_GROUP_OV("switch.probecharger", switch_config_keys, SwitchConfigT, probecharger_ov,
                 switch_config_changed));


static void switch_config_changed(const ConfigTable::Group *g, const void *cfg)
{
    Switch *s = SwitchPool::find(get_checksum(switch_names[g - switch_pool_config_groups]));
    if(s != nullptr) s->configure(*(const SwitchConfigT *)cfg);
}

const SwitchConfigT &switch_light_config()
{
    return ConfigTable::config<SwitchConfigT>(&switch_pool_config_groups[2]);
}

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
        if(s->get_subcode() != gcode->subcode()) continue;
        if(s->get_on_mcode() == gcode->m()) s->on_gcode(gcode);
        else if(s->get_off_mcode() == gcode->m()) s->off_gcode(gcode);
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
    for (int i = 0; i < switch_count; i++) {
        const SwitchConfigT &cfg = ConfigTable::config<SwitchConfigT>(&switch_pool_config_groups[i]);
        if (!cfg.enable) continue;
        Switch *controller = new Switch(get_checksum(switch_names[i]));
        switches.push_back(controller);
        THEKERNEL->add_module(controller);
        controller->load_config(cfg);
    }

    // the registry holds the address of each slot, so the vector must never grow again
    if(!codes.empty()) return;
    codes.resize(switches.size() * 2);
    for(Switch *s : switches) {
        claim(s->get_on_mcode(), s->get_subcode());
        claim(s->get_off_mcode(), s->get_subcode());
    }
}





