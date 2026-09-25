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
#include "Watch.h"
#include "Pin.h"
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
        // steps/sec of the longest axis now: the cap, while braking, is the rate that is being run
        float path_rate() const
        {
            int64_t r= path.steps_per_tick;
            if(state_ == BRAKING && hold_rate_ < r) r= hold_rate_;
            return state_ == HELD ? 0.0F : STEPTICKER_FROMFP(r) * frequency;
        }

        void set_watch(Watch *w) { watch= w; }
        bool watching() const { return watch != nullptr; }

        struct Limit { Pin pin; uint8_t motor; bool at_end; bool at_max; };
        void set_limits(const Limit *l, uint8_t count, uint16_t hyst);
        bool limit_hit() const { return limit_tripped; }
        void clear_limit() { limit_tripped= false; limit_seen= false; }

        // a hold or a stop brakes the path to HELD; whether the block may then be resumed is all the ticker remembers
        enum Motion { IDLE, MOVING, BRAKING, HELD };
        Motion motion() const { return state_; }
        bool resumable() const { return resumable_; }
        bool paused() const { return paused_; }
        int64_t hold_rate() const { return hold_rate_; }   // the cap, 2.62 steps per tick of the longest axis

        void hold(bool on);   // on: brake, and no block starts until off
        void stop();          // brake, and the rest is not to be resumed
        bool take_held(uint32_t done[], uint8_t n);   // HELD only: what the block ran
        uint32_t held_path() const;                   // of the longest axis
        void release();                               // HELD -> IDLE, the queue is sorted out

        void step_tick (void);
        // cycles the tick has cost so far, while `prof on` has the counting switched in
        volatile uint32_t isr_cycles{0};
        volatile uint32_t isr_ticks{0};
        bool counting() const { return counting_; }
        void count(bool on) { isr_cycles= 0; isr_ticks= 0; counting_= on; }
        void handle_finish (void);
        void start();

        // whatever setup the block should register this to know when it is done
        std::function<void()> finished_fnc{nullptr};

    private:
        static StepTicker *instance;
        static void _TIMER0_isr(void);
        static void _TIMER1_isr(void);

        bool start_next_block();
        Motion check_watch();
        void check_limits();

        float frequency;
        uint32_t period;
        std::array<StepperMotor*, k_max_actuators> motor;
        uint32_t unstep;

        Watch *watch{nullptr};
        Limit    limits[k_max_actuators * 2];
        uint8_t  n_limits{0};
        uint16_t limit_hysteresis{0};
        bool     limit_seen{false};
        uint8_t  limit_idx{0};
        int32_t  limit_at_step{0};
        volatile bool limit_tripped{false};
        Block *current_block;
        uint32_t current_tick{0};

        // running state of the block being ticked, 2.62 fixed point rates scaled to each motor
        struct {
            int64_t steps_per_tick;
            int64_t counter;
            int64_t acceleration_change;
            int64_t deceleration_change;
            int64_t plateau_rate;
            uint32_t steps;
            uint32_t step_count;
        } path;
        struct {
            uint32_t steps_to_move; // 0: not moving in this block, or done
            uint32_t step_count;
            uint32_t ratio;
        } state[k_max_actuators];

        void brake(bool may_resume);

        volatile Motion state_{IDLE};
        volatile bool resumable_{true};
        volatile bool paused_{false};   // no block starts while a hold is on
        volatile bool counting_{false};
        int64_t hold_rate_{0};

        uint8_t num_motors;
};