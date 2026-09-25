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

        void calculate_trapezoid( float entry_speed, float exit_speed );

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);
        float max_exit_speed();
        void debug() const;
        void ready() { is_ready= true; }
        void clear();

        uint32_t resume_at{0};
        float remaining_mm() const;
        void set_ratios();

        uint32_t steps_event_count() const; // steps of the longest axis
        float nominal_rate() const { return steps_event_count() * nominal_speed / millimeters; } // steps per second


    private:
        float max_allowable_speed( float acceleration, float target_velocity, float distance);
        void prepare(float initial_rate, float maximum_rate, float final_rate, float accel_distance, float decel_distance);


    public:
        std::array<uint32_t, k_max_actuators> steps; // Number of steps for each axis for this block
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float entry_speed;
        float exit_speed;
        float acceleration;       // the acceleration for this block

        float max_entry_speed;
        unsigned int line;

        uint8_t direction_bits;   // one bit per motor

        // the trapezoid of the longest axis, in steps/s
        struct {
            float entry_rate;      // steps/s at the start
            float plateau_rate;    // steps/s once it is up to speed
            float exit_rate;       // steps/s at the end
            uint32_t accel_steps;  // steps spent getting to the plateau
            uint32_t decel_steps;  // steps spent coming off it
        } ramp;
        uint32_t ratio[k_max_actuators]; // steps[m] / steps_event_count in 0.32 fixed point, 0 for the longest axis
        uint64_t first_owed[k_max_actuators];

        static uint8_t n_actuators;

        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction
            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached
            bool is_ready:1;
            bool primary_axis:1;                 // set if this move is a primary axis
            bool cutting:1;                      // G1/G2/G3: the laser fires only on these

            uint16_t s_value:12;                 // for laser 1.11 Fixed point
        };
        volatile bool is_ticking;
        volatile bool locked;
};
#pragma pack(pop)
