/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURECONTROLPOOL_H
#define TEMPERATURECONTROLPOOL_H

#include <cstdint>
#include <vector>

#include "libs/McodeRegistry.h"

class TemperatureControl;
class Gcode;
struct pad_temperature;

class TemperatureControlPool {
    public:
        void load_tools();

        static TemperatureControl *find(uint16_t name);
        static bool get_temperature(uint16_t name, struct pad_temperature *t);
        static void poll(std::vector<struct pad_temperature> &v);

    private:
        void report_temperature(Gcode *gcode);
        void sensor_settings_gcode(Gcode *gcode);
        void claim(uint16_t code);

        // one slot per distinct get_m_code the controllers asked for
        std::vector<McodeRegistry::Mcode> report_codes;
        size_t used= 0;
        McodeRegistry::Mcode m305;

        static std::vector<TemperatureControl *> controls;
};



#endif
