/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "StepTicker.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "Logging.h"
#include "Block.h"
#include "Conveyor.h"
#include "StepCompress.h"
#include "DeferredWake.h"

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>
#include <stdlib.h>
#include "modules/robot/MachineTask.h"

#ifdef STEPTICKER_DEBUG_PIN
// debug pins, only used if defined in src/makefile
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

StepTicker *StepTicker::instance;

static StepStream step_stream;

StepTicker::StepTicker() : stream(step_stream)
{
    timer_hz= SystemCoreClock / 4.0F;
    inv_timer_hz2= 1.0F / (timer_hz * timer_hz);
}

void StepTicker::init()
{
    instance = this;
    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM0->TCR = 0;              // Disable interrupt

    LPC_SC->PCONP |= (1 << 2);      // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 5;              // match on Mr0, stop on match
    LPC_TIM1->TCR = 0;              // Disable interrupt

    // Default start values
    this->set_frequency(100000);
    this->set_unstep_time(100);

    this->unstep = 0;
    this->num_motors = 0;

    this->state_ = IDLE;
    this->current_block = nullptr;

    #ifdef STEPTICKER_DEBUG_PIN
    // setup debug pin if defined
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

//called when everything is setup and interrupts can start
#if defined(__arm__)
// core cycle counter
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)

struct IsrClock {
    StepTicker &t; uint32_t start; bool on;
    explicit IsrClock(StepTicker &ticker) : t(ticker), start(0), on(ticker.counting())
    {
        if(on) start= DWT_CYCCNT;
    }
    ~IsrClock()
    {
        if(!on) return;
        t.isr_cycles += DWT_CYCCNT - start;
        t.isr_ticks++;
    }
};
#endif

void StepTicker::start()
{
#if defined(__arm__)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA;   // the cycle counter for the interrupt
    DWT_CYCCNT= 0;
    DWT_CTRL |= 1;                                // CYCCNTENA
#endif
    NVIC_SetVector(TIMER0_IRQn, (uintptr_t)&_TIMER0_isr);
    NVIC_SetVector(TIMER1_IRQn, (uintptr_t)&_TIMER1_isr);

    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
    current_tick= 0;
}

void StepTicker::arm(uint32_t ticks)
{
    LPC_TIM0->MR0= ticks;
    LPC_TIM0->TC= 0;
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    StepCompress::configure(this->timer_hz, 0.002F);   // an entry may stray 0.2% of its interval
    this->period = floorf(timer_hz / frequency);
    LPC_TIM0->MR0 = this->period;
    LPC_TIM0->TCR = 3;  // Reset
    LPC_TIM0->TCR = 1;  // start
}

// Set the reset delay, must be called after set_frequency
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf(timer_hz * (microseconds / 1000000.0F));
    LPC_TIM1->MR0 = delay;

    // TODO check that the unstep time is less than the step period, if not slow down step ticker
}

// Reset step pins on any motor that was stepped
void StepTicker::unstep_tick()
{
    uint32_t bits = this->unstep;
    this->unstep = 0;
    for (uint8_t i = 0; bits != 0; i++, bits >>= 1) {
        if(bits & 1) this->motor[i]->unstep();
    }
}

// The actual interrupt handler where we do all the work
void StepTicker::_TIMER0_isr(void)
{
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;
    instance->step_tick();
}

void StepTicker::_TIMER1_isr(void)
{
    LPC_TIM1->IR |= 1 << 0;
    instance->unstep_tick();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::handle_finish (void)
{
    // all moves finished signal block is finished
    if(finished_fnc) finished_fnc();
}

// step clock
void StepTicker::set_limits(const Limit *l, uint8_t count, uint16_t hyst)
{
    for (uint8_t i = 0; i < count; i++) limits[i]= l[i];
    limit_hysteresis= hyst;
    n_limits= count;
    limit_seen= false;
    limit_tripped= false;
}

// stops dead rather than ramping: the travel left past the switch is unknown
// the distance into a switch adds up from its first edge for as long as it stays pressed, so
// creeping into it block by block trips like one move would
StepTicker::Motion StepTicker::check_limits()
{
    if(limit_seen && !limits[limit_idx].pin.get()) limit_seen= false;

    uint8_t closing= n_limits;
    for (uint8_t i = 0; i < n_limits; i++) {
        const Limit &l= limits[i];
        if(!motor[l.motor]->is_moving() || !l.pin.get()) continue;
        if(l.at_end && motor[l.motor]->which_direction() == l.at_max) continue;
        closing= i;
        break;
    }
    if(closing == n_limits) return state_;

    if(!limit_seen || closing != limit_idx) {
        limit_seen= true;
        limit_idx= closing;
        limit_at_step= motor[limits[closing].motor]->get_current_step();
    }
    const Limit &l= limits[limit_idx];
    int32_t into= motor[l.motor]->get_current_step() - limit_at_step;
    if(!l.at_end) {
        into= abs(into);
    } else if(!l.at_max) {
        into= -into;
    }

    if(into < limit_hysteresis)
        return state_;

    stop();
    limit_tripped= true;
    machine_task.halt(HARD_LIMIT, "hard limit");
    return state_;
}

void StepTicker::brake(bool may_resume)
{
    if(state_ == BRAKING || state_ == HELD) {
        if(!may_resume) resumable_= false;   // an approach that hit its switch must not be resumed
        return;
    }
    if(state_ != MOVING) return;

    resumable_= may_resume;
    state_= BRAKING;
    braking_written= false;
    defer_wake();
}

void StepTicker::start_brake()
{
    if(last_interval == 0 || path_decel <= 0) {
        return;
    }
    braking_written= true;

    float v= timer_hz / (float)last_interval;
    brake_v2= v * v;
    brake_c= (float)last_interval;   // what the Newton step refines from
    brake_dv2= 2.0F * (float)path_decel;
}

void StepTicker::hold(bool on)
{
    paused_= on;
    if(on) {
        brake(true);
    }else{
        defer_wake();
    }
}

void StepTicker::stop()
{
    brake(false);
}


bool StepTicker::take_held(uint32_t done[], uint8_t n)
{
    if(state_ != HELD) return false;

    for (uint8_t m = 0; m < n && m < num_motors; m++) done[m]= state[m].step_count;
    return true;
}

uint32_t StepTicker::held_path() const
{
    return state_ == HELD ? path.step_count : 0;
}

void StepTicker::release()
{
    if(state_ != HELD) {
        return;
    }

    state_= IDLE;

    resumable_= true;
    braking_written= false;

    current_block= nullptr;
    path.step_count= 0;
    for (uint8_t m = 0; m < num_motors; m++) {
        state[m].steps_to_move= 0;
        state[m].step_count= 0;
    }
}

uint64_t StepTicker::owed_at(uint32_t j, uint32_t ratio)
{
    if(ratio == 0) {
        return (uint64_t)j << 32; // this motor is the path
    }

    uint64_t num= ((uint64_t)j << 32) - (1ULL << 31);
    uint64_t hi= num / ratio;
    uint64_t rem= num % ratio;
    return (hi << 32) + ((rem << 32) / ratio);
}

// at_steps is taken on the first asserted tick, so the hysteresis does not bias it
StepTicker::Motion StepTicker::check_watch()
{
    if(!watch->inputs.any()) {
        watch->seen= false;
        return state_;
    }

    if(!watch->seen) {
        watch->seen= true;
        watch->witnessed= watch->witness.any();
        for (uint8_t m = 0; m < num_motors; m++) watch->at_steps[m]= motor[m]->get_current_step();
    }

    bool travelled= false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if((watch->motors & (1 << m)) && abs(motor[m]->get_current_step() - watch->at_steps[m]) >= watch->hysteresis) travelled= true;
    }
    if(!travelled) return state_;

    watch->hit= true;
    if(!watch->observe) stop();
    return state_;
}

void StepTicker::step_tick (void)
{
#if defined(__arm__)
    IsrClock clock(*this);
#endif

    uint32_t next= run_tick();
    arm(next != 0 ? next : period);
}

// returns the ticks until the next step, or 0 to come back at the polling interval
inline uint32_t StepTicker::run_tick (void)
{
    //SET_STEPTICKER_DEBUG_PIN(state_ != IDLE ? 1 : 0);

    Motion motion= state_;

    // BRAKING with nothing left to play is already a stand: report it as HELD
    if(motion == BRAKING && current_block == nullptr && stream.empty()) {
        motion= HELD;
        state_= HELD;
        defer_wake();
    }

    // local copy of state_
    motion= state_;
    if(motion == IDLE || motion == HELD) THECONVEYOR.drop_queue();
    if(motion == HELD) return 0;

    if(motion == BRAKING && !braking_written) {
        start_brake();
    }

    if(stream.at_mark()) {
        if(paused_ && motion == IDLE) return 0;
        current_block= THECONVEYOR.take_block(stream.mark());
        path_decel= stream.mark_decel();
        stream.take_mark();
        if(!start_next_block()) return 0;

        if(motion != BRAKING) {
            motion= MOVING;
            state_= MOVING;
        }

    }

    if(motion == IDLE) {
        if(current_block == nullptr) return 0;
        motion= MOVING;
        state_= MOVING;
    }

    if(machine_task.is_halted()) {
        stream.clear();
        state_= IDLE;
        current_tick = 0;
        current_block= nullptr;
        return 0;
    }

    // 0 is the stream saying it has nothing: a real interval is never 0, take() floors it at
    // 1. The mark case was handled above, so this is the ring having run dry.
    uint32_t ticks= stream.take();

    if(ticks == 0) {
        if(!ring_low) {
            defer_wake();
            ring_low= true;
        }
        if(watch != nullptr && !watch->hit) {
            check_watch();
        }
        if(n_limits != 0 && !limit_tripped) {
            check_limits();
        }
        return 0;
    }

    // A brake takes its step from the stream as usual, so the path and the blocks keep their
    // bookkeeping; only the interval is its own. v^2 reaching zero is the stand.
    if(motion == BRAKING && braking_written) {
        brake_v2-= brake_dv2;
        if(brake_v2 <= 0.0F) {
            for (uint8_t m = 0; m < num_motors; m++) motor[m]->stop_moving();
            current_tick= 0;
            state_= HELD;
            defer_wake();
            return 0;
        }
    }
    last_interval= ticks;
    return issue_step(ticks, motion);
}

inline uint32_t StepTicker::issue_step(uint32_t ticks, Motion motion)
{
    ++path.step_count;
    uint64_t here= (uint64_t)path.step_count << 32;

    bool still_moving= false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if(state[m].steps_to_move == 0) continue; // not active

        if(state[m].owed_at <= here) {
            ++state[m].step_count;

            bool ismoving= motor[m]->step(); // returns false if the moving flag was set to false externally (probes, endstops etc)
            unstep |= 1 << m;

            if(!ismoving || state[m].step_count == state[m].steps_to_move) {
                state[m].steps_to_move = 0;
                motor[m]->stop_moving();
            }else{
                // consecutive owed_at values differ by a constant, so the 64-bit divisions
                // it takes are done once when the block starts
                state[m].owed_at+= state[m].owed_step;
            }
        }

        if(motor[m]->is_moving()) still_moving= true;
    }

    current_tick++;

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if(unstep != 0) {
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }

    if(motion == BRAKING && braking_written) {
        brake_c*= (3.0F - brake_v2 * brake_c * brake_c * inv_timer_hz2) * 0.5F;
        ticks= (uint32_t)brake_c;
        if(ticks < 1) ticks= 1;
        last_interval= ticks;
    }

    bool low= stream.used() < StepStream::k_entries / 2;
    if(low && !ring_low) {
        defer_wake();
    }
    ring_low= low;

    if(watch != nullptr && !watch->hit) {
        motion= check_watch();
    }
    if(n_limits != 0 && !limit_tripped) {
        motion= check_limits();
    }


    // see if any motors are still moving
    if(!still_moving) {
        //SET_STEPTICKER_DEBUG_PIN(0);
        current_tick = 0;

        if(current_block != nullptr) {
            THECONVEYOR.block_finished();
            current_block= nullptr;
        }

        if(stream.at_mark()) {
            return 0;
        }

        if(motion == BRAKING) {
            state_= HELD;
        }else{
            state_= IDLE;
        }
        defer_wake();
    }

    return stream.empty() ? 0 : ticks;
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    uint32_t from= current_block->resume_at;

    for (uint8_t m = 0; m < num_motors; m++) {
        state[m].steps_to_move= current_block->steps[m];
        state[m].step_count= 0;
        if(state[m].steps_to_move == 0) continue;
        state[m].ratio= current_block->ratio[m];
        state[m].owed_step= (state[m].ratio == 0)
                            ? ((uint64_t)1 << 32)
                            : owed_at(2, state[m].ratio) - owed_at(1, state[m].ratio);

        if(from == 0) {
            state[m].step_count= 0;
            state[m].owed_at= current_block->first_owed[m];   // worked out by the planner
        }else{
            uint32_t owed;
            if(state[m].ratio == 0) {
                owed= from;
            }else{
                owed= (uint32_t)((((uint64_t)from * state[m].ratio) + (1ULL << 31)) >> 32);
            }
            if(owed > state[m].steps_to_move) {
                owed= state[m].steps_to_move;
            }
            state[m].step_count= owed;
            state[m].owed_at= owed_at(owed + 1, state[m].ratio);

            uint64_t here= (uint64_t)from << 32;
            while(state[m].owed_at <= here && state[m].step_count < state[m].steps_to_move) {
                ++state[m].step_count;
                state[m].owed_at+= state[m].owed_step;
            }
        }

        ok= true; // mark at least one motor is moving
        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction((current_block->direction_bits >> m) & 1);
        motor[m]->start_moving(); // also let motor know it is moving now
    }

    path.step_count= from;
    current_tick= 0;

    if(ok) {
        //SET_STEPTICKER_DEBUG_PIN(1);
        return true;

    }else{
        // this is an edge condition that should never happen, but we need to discard this block if it ever does
        // basically it is a block that has zero steps for all motors
        THECONVEYOR.block_finished();
        current_block= nullptr;
    }

    return false;
}


// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
