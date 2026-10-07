/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstdint>
#include "ActuatorCoordinates.h"

#pragma pack(push, 4) // word aligned: LDRD/STRD work, no int64 padding
class Block {
    public:
        Block();

        static void init(uint8_t);

        void debug() const;
        void ready() { is_ready= true; }
        void clear();

        // 0.32 fraction, rounded up so the last step is never short; 0 is the longest axis.
        // A byte at a time, so the interrupt gets by on the core's 32-bit divide
        static uint32_t share_of(uint32_t steps, uint32_t longest)
        {
            if(steps >= longest) return 0;
            if(longest >= (1UL << 24)) {
                return (uint32_t)((((uint64_t)steps << 32) + longest - 1) / longest);
            }
            uint32_t rem= steps, q= 0;
            for (uint8_t i = 0; i < 4; i++) {
                rem<<= 8;
                q= (q << 8) | (rem / longest);
                rem%= longest;
            }
            return rem != 0 ? q + 1 : q;
        }

        uint32_t steps_event_count() const; // steps of the longest axis
        float nominal_rate() const { return steps_event_count() * nominal_speed / millimeters; } // steps per second


    public:
        std::array<uint32_t, k_max_actuators> steps; // Number of steps for each axis for this block
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float acceleration;       // the acceleration for this block

        float max_entry_speed;
        uint32_t mark;

        uint8_t direction_bits;   // one bit per motor


        static uint8_t n_actuators;

        struct {
            bool is_ready:1;
            bool primary_axis:1;                 // set if this move is a primary axis
            bool cutting:1;                      // G1/G2/G3: the laser fires only on these

            uint16_t s_value:12;                 // for laser 1.11 Fixed point
        };
};
#pragma pack(pop)
