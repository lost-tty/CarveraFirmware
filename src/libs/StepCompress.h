/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>

class StepStream;

class StepCompress
{
public:
    static void configure(float timer_hz, float tolerance);
    static uint32_t ramp(StepStream &out, float from, float to, uint32_t steps, uint32_t done= 0);
    static uint32_t plateau(StepStream &out, float rate, uint32_t steps);
    static float interval_at(float from, float to, uint32_t steps, uint32_t k);

private:
    static float timer_hz_;
    static float tolerance_;
};
