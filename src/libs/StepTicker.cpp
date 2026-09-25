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
#include "DeferredWake.h"

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>
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
    StepTicker &t; uint32_t start;
    explicit IsrClock(StepTicker &ticker) : t(ticker), start(DWT_CYCCNT) {}
    ~IsrClock() { t.isr_cycles += DWT_CYCCNT - start; t.isr_ticks++; }
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

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock / 4.0F) / frequency); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    LPC_TIM0->TCR = 3;  // Reset
    LPC_TIM0->TCR = 1;  // start
}

// Set the reset delay, must be called after set_frequency
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf((SystemCoreClock / 4.0F) * (microseconds / 1000000.0F)); // SystemCoreClock/4 = Timer increments in a second
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
void StepTicker::check_limits()
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
    if(closing == n_limits) return;

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
        return;

    for (uint8_t m = 0; m < num_motors; m++) motor[m]->stop_moving();
    limit_tripped= true;
    machine_task.halt(HARD_LIMIT, "hard limit");
}

void StepTicker::brake(bool may_resume)
{
    if(state_ == BRAKING || state_ == HELD) {
        if(!may_resume) resumable_= false;   // an approach that hit its switch must not be resumed
        return;
    }
    if(state_ != MOVING) return;

    resumable_= may_resume;
    hold_rate_= path.steps_per_tick;   // the cap starts at the rate the path has now
    state_= BRAKING;
}

void StepTicker::hold(bool on)
{
    paused_= on;
    if(on) brake(true);
}

void StepTicker::stop()
{
    brake(false);
}

uint32_t StepTicker::held_path() const
{
    return state_ == HELD ? path.step_count : 0;
}

bool StepTicker::take_held(uint32_t done[], uint8_t n)
{
    if(state_ != HELD) return false;

    for (uint8_t m = 0; m < n && m < num_motors; m++) done[m]= state[m].step_count;
    return true;
}

void StepTicker::release()
{
    if(state_ == HELD) state_= IDLE;
}

// the steps a motor has made at this point of the path: path steps with a 0.32 fraction, times the
// motor's ratio, to the nearest step, so it lands on its last step with the path's last step
static inline uint32_t owed(uint32_t path_steps, uint32_t path_frac, uint32_t ratio)
{
    uint64_t v= (uint64_t)path_steps * ratio + (((uint64_t)path_frac * ratio) >> 32) + (1u << 31);
    return (uint32_t)(v >> 32);
}

// at_steps is taken on the first asserted tick, so the hysteresis does not bias it
void StepTicker::check_watch()
{
    if(!watch->inputs.any()) {
        watch->seen= false;
        return;
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
    if(!travelled) return;

    watch->hit= true;
    if(!watch->observe) stop();
}

void StepTicker::step_tick (void)
{
#if defined(__arm__)
    IsrClock clock(*this);
#endif

    //SET_STEPTICKER_DEBUG_PIN(state_ != IDLE ? 1 : 0);

    // brake() runs in task context and cannot mask this interrupt: it may have stamped BRAKING onto
    // a ticker whose last block ended in the same tick
    if(state_ == BRAKING && current_block == nullptr) state_= IDLE;

    // a flush lands only while the ticker stands, so a brake runs out before the queue goes
    if(state_ == IDLE || state_ == HELD) THECONVEYOR.drop_queue();
    if(state_ == HELD) return;
    if(state_ == IDLE) {
        if(paused_) return;
        if(!THECONVEYOR.get_next_block(&current_block)) return;
        if(!start_next_block()) return;
        state_= MOVING;
    }

    if(machine_task.is_halted()) {
        state_= IDLE;
        current_tick = 0;
        current_block= nullptr;
        return;
    }

    if(watch != nullptr && !watch->hit) check_watch();
    // check limits every 8 ticks
    if(n_limits != 0 && !limit_tripped && (current_tick & 7) == 0) check_limits();

    if(state_ == BRAKING) {
        int64_t a= current_block->ramp.brake_change;
        hold_rate_= hold_rate_ > a ? hold_rate_ - a : 0;

        if(hold_rate_ == 0) {
            for (uint8_t m = 0; m < num_motors; m++) motor[m]->stop_moving();
            current_tick= 0;
            state_= HELD;
            defer_wake();
            return;
        }
    }

    bool accel_end= current_tick == current_block->accelerate_until && current_block->accelerate_until != 0;
    bool decel_start= current_tick == current_block->decelerate_after;

    path.steps_per_tick += path.acceleration_change;

    if(accel_end) { // done accelerating: plateau, unless deceleration starts right here
        path.acceleration_change = 0;
        if(current_block->decelerate_after < current_block->total_move_ticks && !decel_start) {
            path.steps_per_tick = path.plateau_rate;
        }
    }
    if(decel_start) {
        path.acceleration_change = path.deceleration_change;
    }

    // protect against rounding errors and such
    if(path.steps_per_tick <= 0) {
        if(state_ != BRAKING) path.counter = STEPTICKER_FPSCALE; // we force completion this step by setting to 1.0
        path.steps_per_tick = 0;
    }

    int64_t rate= path.steps_per_tick;
    if(state_ == BRAKING && hold_rate_ < rate) rate= hold_rate_;
    path.counter += rate;

    if(path.counter >= STEPTICKER_FPSCALE) { // >= 1.0 step time
        path.counter -= STEPTICKER_FPSCALE;
        ++path.step_count;
    }
    uint32_t frac= (uint32_t)(path.counter >> 30);

    bool still_moving= false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if(state[m].steps_to_move == 0) continue; // not active

        uint32_t due= state[m].ratio == 0 ? path.step_count : owed(path.step_count, frac, state[m].ratio);
        if(due > state[m].steps_to_move) due= state[m].steps_to_move;

        if(due > state[m].step_count) {
            ++state[m].step_count;

            bool ismoving= motor[m]->step(); // returns false if the moving flag was set to false externally (probes, endstops etc)
            unstep |= 1 << m;

            if(!ismoving || state[m].step_count == state[m].steps_to_move) {
                state[m].steps_to_move = 0;
                motor[m]->stop_moving();
            }
        }

        if(motor[m]->is_moving()) still_moving= true;
    }

    // do this after so we start at tick 0
    current_tick++; // count number of ticks

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if(unstep != 0) {
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }


    // see if any motors are still moving
    if(!still_moving) {
        //SET_STEPTICKER_DEBUG_PIN(0);

        // all moves finished
        current_tick = 0;

        // get next block
        // do it here so there is no delay in ticks
        THECONVEYOR.block_finished();

        bool running= THECONVEYOR.get_next_block(&current_block) && start_next_block();

        if(running) return;

        current_block= nullptr;

        if(state_ == BRAKING) {
            state_= HELD;
            defer_wake();
        }else{
            state_= IDLE;
        }
    }
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    path.steps= 0;
    for (uint8_t m = 0; m < num_motors; m++) {
        state[m].steps_to_move= current_block->steps[m];
        if(state[m].steps_to_move == 0) continue;
        state[m].step_count= 0;
        state[m].ratio= current_block->ratio[m];
        if(state[m].steps_to_move > path.steps) path.steps= state[m].steps_to_move;

        ok= true; // mark at least one motor is moving
        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction((current_block->direction_bits >> m) & 1);
        motor[m]->start_moving(); // also let motor know it is moving now
    }

    path.step_count= 0;
    path.counter= 0;
    path.steps_per_tick= current_block->ramp.steps_per_tick;
    path.acceleration_change= current_block->ramp.acceleration_change;
    path.deceleration_change= current_block->ramp.deceleration_change;
    path.plateau_rate= current_block->ramp.plateau_rate;
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
