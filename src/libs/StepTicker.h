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
#include "StepStream.h"

class StepperMotor;
class Block;

// handle 2.62 Fixed point
#define STEPTICKER_FPSCALE (1LL<<62)
#define STEPTICKER_FROMFP(x) ((float)(x)/STEPTICKER_FPSCALE)

class StepTicker{
    public:
        StepTicker();
        void init();
        void set_frequency( float frequency );
        void set_unstep_time( float microseconds );
        int register_motor(StepperMotor* motor);
        float get_frequency() const { return frequency; }
        void unstep_tick();
        const Block *get_current_block() const { return current_block; }
        // steps/sec of the longest axis now, from the interval the last step was armed with
        float path_rate() const
        {
            if(state_ == HELD || last_interval == 0) {
                return 0.0F;
            }
            return timer_hz / (float)last_interval;
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

        void hold(bool on);   // on: brake, and no block starts until off
        void stop();          // brake, and the rest is not to be resumed
        bool take_held(uint32_t done[], uint8_t n);   // HELD only: what the block ran
        void release();                               // HELD -> IDLE, the queue is sorted out
        uint32_t held_path() const;                   // path steps the held block ran

        StepStream &steps() { return stream; }


        static uint64_t owed_at(uint32_t j, uint32_t ratio);

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
        void arm(uint32_t ticks);
        __attribute__((always_inline)) inline uint32_t run_tick(void);
        __attribute__((always_inline)) inline uint32_t issue_step(uint32_t ticks, Motion motion);
        Motion check_watch();
        Motion check_limits();

        float frequency;
        float timer_hz;
        uint32_t period;
        uint32_t last_interval{0};
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

        StepStream &stream;
        struct {
            uint32_t step_count;
        } path;
        struct {
            uint32_t steps_to_move; // 0: not moving in this block, or done
            uint32_t step_count;
            uint32_t ratio;
            uint64_t owed_at;
            uint64_t owed_step;     // what owed_at gains per step of this motor
        } state[k_max_actuators];


        void brake(bool may_resume);
        void start_brake();

        volatile Motion state_{IDLE};
        volatile bool resumable_{true};
        volatile bool paused_{false};   // no block starts while a hold is on
        volatile bool counting_{false};
        int32_t path_decel{0};
        volatile bool braking_written{false};
        bool ring_low{false};
        float brake_v2{0.0F};
        float brake_dv2{0.0F};
        float brake_c{0.0F};         // the braking interval in ticks, refined a step at a time
        float inv_timer_hz2{0.0F};   // 1 / timer_hz^2, for that refinement

        uint8_t num_motors;
};