/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "utils/GcodeLine.h"
#include "utils/Parameters.h"

#include <cstdint>
#include <string>
#include <vector>

class StreamOutput;

class GcodeDispatch : public Module
{
public:
    void init();

    virtual void on_module_loaded();
    virtual void on_console_line_received(void *line);

    uint8_t get_modal_command() const { return modal_group_1; }
private:
    void execute(const std::vector<gcode::Word> &words, const std::string &text, StreamOutput *stream, unsigned int line);
    void parameter_statement(const char *p, StreamOutput *stream);
    void fail(StreamOutput *stream, const char *msg);
    void halt();

    Parameters params;
    uint8_t modal_group_1;
    bool homed_check;
};
