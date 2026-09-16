/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <cmath>
#include <string>
#include "Block.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/Logging.h"

#include "mri.h"
#include <inttypes.h>

using std::string;
using std::min;

#define STEP_TICKER_FREQUENCY THEKERNEL->step_ticker.get_frequency()

uint8_t Block::n_actuators= 0;
double Block::fp_scale= 0;

// A block represents a movement, it's length for each stepper motor, and the corresponding acceleration curves.
// It's stacked on a queue, and that queue is then executed in order, to move the motors.
// Most of the accel math is also done in this class
// And GCode objects for use in on_gcode_execute are also help in here

Block::Block()
{
    line = 0;
    clear();
}

void Block::init(uint8_t n)
{
    n_actuators= n;
    fp_scale= (double)STEPTICKER_FPSCALE / pow((double)STEP_TICKER_FREQUENCY, 2.0); // we scale up by fixed point offset first to avoid tiny values
}

void Block::clear()
{
    is_ready            = false;

    this->steps.fill(0);
    direction_bits      = 0;
    nominal_speed       = 0.0F;
    millimeters         = 0.0F;
    entry_speed         = 0.0F;
    exit_speed          = 0.0F;
    acceleration        = 100.0F; // we don't want to get divide by zeroes if this is not set
    accelerate_until    = 0;
    decelerate_after    = 0;
    total_move_ticks    = 0;
    recalculate_flag    = false;
    nominal_length_flag = false;
    max_entry_speed     = 0.0F;
    is_ticking          = false;
    is_g123             = false;
    locked              = false;
    s_value             = 0;

    ramp.steps_per_tick= 0;
    ramp.acceleration_change= 0;
    ramp.deceleration_change= 0;
    ramp.plateau_rate= 0;
    for(int i = 0; i < n_actuators; ++i) ratio[i]= 0;
}

uint32_t Block::steps_event_count() const
{
    uint32_t n= 0;
    for(int i = 0; i < n_actuators; ++i) if(steps[i] > n) n= steps[i];
    return n;
}

void Block::debug() const
{
    printk("%p: steps-X:%lu Y:%lu Z:%lu ", this, this->steps[0], this->steps[1], this->steps[2]);
    for (size_t i = E_AXIS; i < n_actuators; ++i) {
        printk("%c:%lu ", 'A' + i-E_AXIS, this->steps[i]);
    }
    printk("(max:%lu) nominal:r%1.4f/s%1.4f mm:%1.4f acc:%1.2f accu:%lu decu:%lu ticks:%lu entry/max:%1.4f/%1.4f exit:%1.4f primary:%d ready:%d locked:%d ticking:%d recalc:%d nomlen:%d time:%f\r\n",
                               steps_event_count(),
                               nominal_rate(),
                               this->nominal_speed,
                               this->millimeters,
                               this->acceleration,
                               this->accelerate_until,
                               this->decelerate_after,
                               this->total_move_ticks,
                               this->entry_speed,
                               this->max_entry_speed,
                               this->exit_speed,
                               this->primary_axis,
                               this->is_ready,
                               this->locked,
                               this->is_ticking,
                               recalculate_flag ? 1 : 0,
                               nominal_length_flag ? 1 : 0,
                               total_move_ticks/STEP_TICKER_FREQUENCY
                              );
}


/* Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
//                                +--------+ <- nominal_rate
//                               /          \
// nominal_rate*entry_factor -> +            \
//                              |             + <- nominal_rate*exit_factor
//                              +-------------+
//                                  time -->
*/
void Block::calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (is_ticking) return;

    uint32_t steps_event_count = this->steps_event_count();
    float nominal_rate = this->nominal_rate();
    float initial_rate = nominal_rate * (entryspeed / this->nominal_speed); // steps/sec
    float final_rate = nominal_rate * (exitspeed / this->nominal_speed);
    //printf("Initial rate: %f, final_rate: %f\n", initial_rate, final_rate);
    // How many steps ( can be fractions of steps, we need very precise values ) to accelerate and decelerate
    // This is a simplification to get rid of rate_delta and get the steps/s² accel directly from the mm/s² accel
    float acceleration_per_second = (this->acceleration * steps_event_count) / this->millimeters;

    float maximum_possible_rate = sqrtf( ( steps_event_count * acceleration_per_second ) + ( ( powf(initial_rate, 2) + powf(final_rate, 2) ) / 2.0F ) );

    //printf("id %d: acceleration_per_second: %f, maximum_possible_rate: %f steps/sec, %f mm/sec\n", this->id, acceleration_per_second, maximum_possible_rate, maximum_possible_rate/100);

    // Now this is the maximum rate we'll achieve this move, either because
    // it's the higher we can achieve, or because it's the higher we are
    // allowed to achieve
    float maximum_rate = std::min(maximum_possible_rate, nominal_rate);

    // Now figure out how long it takes to accelerate in seconds
    float time_to_accelerate = ( maximum_rate - initial_rate ) / acceleration_per_second;

    // Now figure out how long it takes to decelerate
    float time_to_decelerate = ( final_rate -  maximum_rate ) / -acceleration_per_second;

    // Now we know how long it takes to accelerate and decelerate, but we must
    // also know how long the entire move takes so we can figure out how long
    // is the plateau if there is one
    float plateau_time = 0;

    // Only if there is actually a plateau ( we are limited by nominal_rate )
    if(maximum_possible_rate > nominal_rate) {
        // Figure out the acceleration and deceleration distances ( in steps )
        float acceleration_distance = ( ( initial_rate + maximum_rate ) / 2.0F ) * time_to_accelerate;
        float deceleration_distance = ( ( maximum_rate + final_rate ) / 2.0F ) * time_to_decelerate;

        // Figure out the plateau steps
        float plateau_distance = steps_event_count - acceleration_distance - deceleration_distance;

        // Figure out the plateau time in seconds
        plateau_time = plateau_distance / maximum_rate;
    }

    // Figure out how long the move takes total ( in seconds )
    float total_move_time = time_to_accelerate + time_to_decelerate + plateau_time;
    //puts "total move time: #{total_move_time}s time to accelerate: #{time_to_accelerate}, time to decelerate: #{time_to_decelerate}"

    // We now have the full timing for acceleration, plateau and deceleration,
    // yay \o/ Now this is very important these are in seconds, and we need to
    // round them into ticks. This means instead of accelerating in 100.23
    // ticks we'll accelerate in 100 ticks. Which means to reach the exact
    // speed we want to reach, we must figure out a new/slightly different
    // acceleration/deceleration to be sure we accelerate and decelerate at
    // the exact rate we want

    // First off round total time, acceleration time and deceleration time in ticks
    uint32_t acceleration_ticks = floorf( time_to_accelerate * STEP_TICKER_FREQUENCY );
    uint32_t deceleration_ticks = floorf( time_to_decelerate * STEP_TICKER_FREQUENCY );
    uint32_t total_move_ticks   = floorf( total_move_time    * STEP_TICKER_FREQUENCY );

    // Now deduce the plateau time for those new values expressed in tick
    //uint32_t plateau_ticks = total_move_ticks - acceleration_ticks - deceleration_ticks;

    // Now we figure out the acceleration value to reach EXACTLY maximum_rate(steps/s) in EXACTLY acceleration_ticks(ticks) amount of time in seconds
    float acceleration_time = acceleration_ticks / STEP_TICKER_FREQUENCY;  // This can be moved into the operation below, separated for clarity, note we need to do this instead of using time_to_accelerate(seconds) directly because time_to_accelerate(seconds) and acceleration_ticks(seconds) do not have the same value anymore due to the rounding
    float deceleration_time = deceleration_ticks / STEP_TICKER_FREQUENCY;

    float acceleration_in_steps = (acceleration_time > 0.0F ) ? ( maximum_rate - initial_rate ) / acceleration_time : 0;
    float deceleration_in_steps =  (deceleration_time > 0.0F ) ? ( maximum_rate - final_rate ) / deceleration_time : 0;

    // we have a potential race condition here as we could get interrupted anywhere in the middle of this call, we need to lock
    // the updates to the blocks to get around it
    this->locked= true;
    // Now figure out the two acceleration ramp change events in ticks
    this->accelerate_until = acceleration_ticks;
    this->decelerate_after = total_move_ticks - deceleration_ticks;

    // We now have everything we need for this block to call a Steppermotor->move method !!!!
    // Theorically, if accel is done per tick, the speed curve should be perfect.
    this->total_move_ticks = total_move_ticks;

    this->exit_speed = exitspeed;

    // prepare the block for stepticker
    this->prepare(initial_rate, maximum_rate, acceleration_in_steps, deceleration_in_steps);

    this->locked= false;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Block::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}

// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->entry_speed != this->max_entry_speed) {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->nominal_length_flag) && (this->max_entry_speed > exit_speed)) {
            float max_entry_speed = max_allowable_speed(-this->acceleration, exit_speed, this->millimeters);

            this->entry_speed = min(max_entry_speed, this->max_entry_speed);

            return this->entry_speed;
        } else
            this->entry_speed = this->max_entry_speed;
    }

    return this->entry_speed;
}


// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > nominal_speed)
        prev_max_exit_speed = nominal_speed;
    if (prev_max_exit_speed > max_entry_speed)
        prev_max_exit_speed = max_entry_speed;

    if (prev_max_exit_speed <= entry_speed) {
        // accel limited
        entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}

float Block::max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if(is_ticking)
        return this->exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (nominal_length_flag)
        return nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-this->acceleration, this->entry_speed, this->millimeters);

    return min(max, nominal_speed);
}

// prepare block for the step ticker, called everytime the block changes
// this is done during planning so does not delay tick generation and step ticker can simply grab the next block during the interrupt
void Block::prepare(float initial_rate, float maximum_rate, float acceleration_in_steps, float deceleration_in_steps)
{
    // acceleration per tick in steps/tick^2, scaled to 2.62 as a double because it is tiny
    double acceleration_per_tick = acceleration_in_steps * fp_scale;
    double deceleration_per_tick = deceleration_in_steps * fp_scale;

    // the ramp the block starts with: accelerating, or decelerating right away, or the plateau
    double acceleration_change = 0;
    if(this->accelerate_until != 0) {
        acceleration_change = acceleration_per_tick;
    } else if(this->decelerate_after == 0) {
        acceleration_change = -deceleration_per_tick;
    }

    ramp.steps_per_tick = (int64_t)round(((double)initial_rate / STEP_TICKER_FREQUENCY) * STEPTICKER_FPSCALE);
    ramp.acceleration_change= (int64_t)round(acceleration_change);
    ramp.deceleration_change= -(int64_t)round(deceleration_per_tick);
    ramp.plateau_rate= (int64_t)round(((double)maximum_rate / STEP_TICKER_FREQUENCY) * STEPTICKER_FPSCALE);
}
