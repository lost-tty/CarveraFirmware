#pragma once

#include "libs/Module.h"
#include "GcodeDispatch.h"

class Gcode;

// M3 and M5 mean the spindle or the laser, depending which head the machine is in.
class ToolHead : public Module {
public:
    void on_module_loaded() override;

    void stop_all();

private:
    void start(Gcode *);
    void stop(Gcode *);

    GcodeDispatch::Mcode m3, m5;
};

extern ToolHead tool_head;
