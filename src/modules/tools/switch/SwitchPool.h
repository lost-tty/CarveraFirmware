/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SWITCHPOOL_H
#define SWITCHPOOL_H

#include <cstdint>
#include <vector>

class Switch;
struct pad_switch;

class SwitchPool{
    public:
        void load_tools();

        static Switch *find(uint16_t name);
        static bool get_state(uint16_t name, struct pad_switch *pad);
        static bool set_state(uint16_t name, bool on);
        static bool set_state(uint16_t name, bool on, float value);

    private:
        static std::vector<Switch *> switches;
};

#endif // SWITCHPOOL_H
