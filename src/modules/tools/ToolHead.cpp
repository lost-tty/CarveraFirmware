#include "ToolHead.h"

#include "libs/Kernel.h"
#include "Gcode.h"
#include "SpindleControl.h"
#include "SwitchPool.h"
#include "SwitchPublicAccess.h"
#ifndef NO_TOOLS_LASER
#include "Laser.h"
#endif

ToolHead tool_head;

void ToolHead::on_module_loaded()
{
    ADD_MCODE(m3, 3, ACTION, ToolHead::start);
    ADD_MCODE(m5, 5, ACTION, ToolHead::stop);
}

void ToolHead::start(Gcode *gcode)
{
#ifndef NO_TOOLS_LASER
    if(THEKERNEL->get_laser_mode()) {
        laser.start(gcode);
        return;
    }
#endif
    if(spindle_control != nullptr) spindle_control->start(gcode);
}

// what M5 and M9 do, for the paths that end a program rather than run one
void ToolHead::stop_all()
{
#ifndef NO_TOOLS_LASER
    if(THEKERNEL->get_laser_mode()) laser.stop(nullptr);
    else
#endif
    if(spindle_control != nullptr) spindle_control->stop(nullptr);

    SwitchPool::set_state(air_checksum, false);
}

void ToolHead::stop(Gcode *gcode)
{
#ifndef NO_TOOLS_LASER
    if(THEKERNEL->get_laser_mode()) {
        laser.stop(gcode);
        return;
    }
#endif
    if(spindle_control != nullptr) spindle_control->stop(gcode);
}
