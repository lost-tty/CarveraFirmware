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

        uint32_t steps_event_count() const; // steps of the longest axis
        float nominal_rate() const { return steps_event_count() * nominal_speed / millimeters; } // steps per second

    private:
        float max_allowable_speed( float acceleration, float target_velocity, float distance);
        void prepare(float initial_rate, float maximum_rate, float acceleration_in_steps, float deceleration_in_steps);

        static double fp_scale; // optimize to store this as it does not change

    public:
        std::array<uint32_t, k_max_actuators> steps; // Number of steps for each axis for this block
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float entry_speed;
        float exit_speed;
        float acceleration;       // the acceleration for this block

        float max_entry_speed;
        unsigned int line;

        // this is tick info needed for this block. applies to all motors
        uint32_t accelerate_until;
        uint32_t decelerate_after;
        uint32_t total_move_ticks;
        uint8_t direction_bits;   // one bit per motor

        // ramp of the longest axis in 2.62 fixed point; each motor scales it by its ratio. The running
        // state lives in StepTicker.
        struct {
            int64_t steps_per_tick;      // at block start
            int64_t acceleration_change; // at block start, signed
            int64_t deceleration_change;
            int64_t plateau_rate;
        } ramp;
        uint32_t ratio[k_max_actuators]; // steps[m] / steps_event_count in 0.32 fixed point, 0 for the longest axis

        static uint8_t n_actuators;

        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction
            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached
            bool is_ready:1;
            bool primary_axis:1;                 // set if this move is a primary axis
            bool is_g123:1;                      // set if this is a G1, G2 or G3
            volatile bool is_ticking:1;          // set when this block is being actively ticked by the stepticker
            volatile bool locked:1;              // set to true when the critical data is being updated, stepticker will have to skip if this is set

            uint16_t s_value:12;                 // for laser 1.11 Fixed point
        };
};
#pragma pack(pop)
