/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once

#include <stdint.h>
#include <array>
#include <bitset>
#include <functional>
#include <atomic>

#include "ActuatorCoordinates.h"
#include "TSRingBuffer.h"

class StepperMotor;
class Block;

// handle 2.62 Fixed point
#define STEPTICKER_FPSCALE (1LL<<62)
#define STEPTICKER_FROMFP(x) ((float)(x)/STEPTICKER_FPSCALE)

class StepTicker{
    public:
        void init();
        void set_frequency( float frequency );
        void set_unstep_time( float microseconds );
        int register_motor(StepperMotor* motor);
        float get_frequency() const { return frequency; }
        void unstep_tick();
        const Block *get_current_block() const { return current_block; }
        float get_trapezoid_rate(int m) const { return STEPTICKER_FROMFP(state[m].steps_per_tick) * frequency; } // steps/sec now

        void step_tick (void);
        void handle_finish (void);
        void start();

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

    private:
        static StepTicker *instance;
        static void _TIMER0_isr(void);
        static void _TIMER1_isr(void);

        bool start_next_block();

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        std::bitset<k_max_actuators> unstep;

        Block *current_block;
        uint32_t current_tick{0};

        // running state of the block being ticked, 2.62 fixed point rates scaled to each motor
        struct {
            int64_t steps_per_tick;
            int64_t counter;
            int64_t acceleration_change;
            int64_t deceleration_change;
            int64_t plateau_rate;
            uint32_t steps_to_move; // 0: not moving in this block, or done
            uint32_t step_count;
        } state[k_max_actuators];

        struct {
            volatile bool running:1;
            uint8_t num_motors:4;
        };
};