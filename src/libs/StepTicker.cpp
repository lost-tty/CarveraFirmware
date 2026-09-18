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

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

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

    this->running = false;
    this->current_block = nullptr;

    #ifdef STEPTICKER_DEBUG_PIN
    // setup debug pin if defined
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

//called when everything is setup and interrupts can start
void StepTicker::start()
{
    NVIC_SetVector(TIMER0_IRQn, (uint32_t)&_TIMER0_isr);
    NVIC_SetVector(TIMER1_IRQn, (uint32_t)&_TIMER1_isr);

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
void StepTicker::step_tick (void)
{
    //SET_STEPTICKER_DEBUG_PIN(running ? 1 : 0);

    // if nothing has been setup we ignore the ticks
    if(!running){
        // check if anything new available
        if(THECONVEYOR.get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue
            if(!running) return;
        }else{
            return;
        }
    }

    if(THEKERNEL->is_halted()) {
        running= false;
        current_tick = 0;
        current_block= nullptr;
        return;
    }

    bool still_moving= false;
    bool accel_end= current_tick == current_block->accelerate_until && current_block->accelerate_until != 0;
    bool decel_start= current_tick == current_block->decelerate_after;
    // foreach motor, if it is active see if time to issue a step to that motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(state[m].steps_to_move == 0) continue; // not active

        state[m].steps_per_tick += state[m].acceleration_change;

        if(accel_end) { // done accelerating: plateau, unless deceleration starts right here
            state[m].acceleration_change = 0;
            if(current_block->decelerate_after < current_block->total_move_ticks && !decel_start) {
                state[m].steps_per_tick = state[m].plateau_rate;
            }
        }
        if(decel_start) {
            state[m].acceleration_change = state[m].deceleration_change;
        }

        // protect against rounding errors and such
        if(state[m].steps_per_tick <= 0) {
            state[m].counter = STEPTICKER_FPSCALE; // we force completion this step by setting to 1.0
            state[m].steps_per_tick = 0;
        }

        state[m].counter += state[m].steps_per_tick;

        if(state[m].counter >= STEPTICKER_FPSCALE) { // >= 1.0 step time
            state[m].counter -= STEPTICKER_FPSCALE; // -= 1.0F;
            ++state[m].step_count;

            // step the motor
            bool ismoving= motor[m]->step(); // returns false if the moving flag was set to false externally (probes, endstops etc)
            // we stepped so schedule an unstep
            unstep |= 1 << m;

            if(!ismoving || state[m].step_count == state[m].steps_to_move) {
                // done
                state[m].steps_to_move = 0;
                motor[m]->stop_moving(); // let motor know it is no longer moving
            }
        }

        // see if any motors are still moving after this tick
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

        if(THECONVEYOR.get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue

        }else{
            current_block= nullptr;
            running= false;
        }
    }
}

// 2.62 value scaled by a 0.32 ratio, 0 meaning 1.0
static inline int64_t scale(int64_t v, uint32_t ratio)
{
    if(ratio == 0) return v;
    int64_t hi= (int64_t)(int32_t)(v >> 32) * ratio;
    uint64_t lo= ((uint64_t)(uint32_t)v * ratio) >> 32;
    return hi + (int64_t)lo;
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    // need to prepare each active motor
    for (uint8_t m = 0; m < num_motors; m++) {
        state[m].steps_to_move= current_block->steps[m];
        if(state[m].steps_to_move == 0) continue;
        state[m].step_count= 0;
        state[m].counter= 0;
        uint32_t ratio= current_block->ratio[m];
        state[m].steps_per_tick= scale(current_block->ramp.steps_per_tick, ratio);
        state[m].acceleration_change= scale(current_block->ramp.acceleration_change, ratio);
        state[m].deceleration_change= scale(current_block->ramp.deceleration_change, ratio);
        state[m].plateau_rate= scale(current_block->ramp.plateau_rate, ratio);

        ok= true; // mark at least one motor is moving
        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction((current_block->direction_bits >> m) & 1);
        motor[m]->start_moving(); // also let motor know it is moving now
    }

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
