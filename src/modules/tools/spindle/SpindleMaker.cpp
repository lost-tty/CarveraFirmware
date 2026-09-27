/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SpindleMaker.h"
#include "GcodeDispatch.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SpindleControl.h"
#include "PWMSpindleControl.h"
#include "AnalogSpindleControl.h"
#include "HuanyangSpindleControl.h"
#include "SpindleConfig.h"
#include "Logging.h"

CONFIG_KEYS(spindle_config_keys, SpindleConfigT, SPINDLE_CONFIG);
static void spindle_config_changed(const ConfigTable::Group *, const void *c)
{
    PWMSpindleControl::configure_active(c);
    AnalogSpindleControl::configure_active(c);
}
CONFIG_GROUPS(spindle_maker_config_groups,
    CFG_GROUP("spindle", spindle_config_keys, SpindleConfigT, spindle_config_changed));


void SpindleMaker::load_spindle(){

    const SpindleConfigT &spindle_config = spindle_cfg();
    // If the spindle module is disabled load no Spindle
    if( !spindle_config.enable ) {
        printk("NOTE: Spindle Module is disabled\n");
        return;
    }

    spindle = NULL;

    // get the two config options that make us able to determine which spindle module we need to load

    // check config which spindle type we need
    if( spindle_config.type == SPINDLE_PWM ) {
        spindle = new PWMSpindleControl();
    } else if ( spindle_config.type == SPINDLE_ANALOG ) {
        spindle = new AnalogSpindleControl();
    } else if ( spindle_config.type == SPINDLE_MODBUS ) {
        if(spindle_config.vfd_type == VFD_HUANYANG) {
            spindle = new HuanyangSpindleControl();
        } else {
            delete spindle;
            printk("ERROR: No valid spindle VFD type defined\n");
        }
    } else {
        delete spindle;
        printk("ERROR: No valid spindle type defined\n");
    }

    // Add the spindle if we successfully initialized one
    if( spindle != NULL) {

        spindle_control = spindle;
        spindle->register_mcodes();

        THEKERNEL->add_module( spindle );
    }

}

