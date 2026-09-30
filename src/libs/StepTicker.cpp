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

static const uint32_t k_min_lead= 32;

StepTicker::StepTicker() : stream(step_stream)
{
    timer_hz= SystemCoreClock / 4.0F;
    inv_timer_hz2= 1.0F / (timer_hz * timer_hz);
    poll_ticks= (int32_t)(timer_hz / 10000.0F);
    dir_lead_ticks= (uint32_t)(timer_hz * 5e-6F);
}

void StepTicker::init()
{
    instance = this;
    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 1;              // interrupt on MR0; the count runs on, MR0 is moved ahead of it
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

    // one port write steps every motor due in a tick, so the pins have to share a port
    step_port= nullptr;
    for (uint8_t m = 0; m < num_motors; m++) {
        step_bit[m]= motor[m]->step_bit();
        if(step_bit[m] == 0) continue;
        if(step_port == nullptr) {
            step_port= motor[m]->step_port();
            step_inv= motor[m]->step_inverting();
        } else if(motor[m]->step_port() != step_port || motor[m]->step_inverting() != step_inv) {
            printk("FATAL: step pins must share one port and polarity, motor %d will not step\n", m);
            step_bit[m]= 0;
        }
    }
}

void StepTicker::arm(uint32_t ticks)
{
    uint32_t next= LPC_TIM0->MR0 + ticks;
    if((int32_t)(next - LPC_TIM0->TC) < (int32_t)k_min_lead) next= LPC_TIM0->TC + k_min_lead;
    LPC_TIM0->MR0= next;
    // MR0 matches TC on equality: a write that lands behind the count fires after the 32-bit wrap
    while((int32_t)(LPC_TIM0->TC - LPC_TIM0->MR0) > 0 && !(LPC_TIM0->IR & 1)) {
        LPC_TIM0->MR0= LPC_TIM0->TC + k_min_lead;
    }
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
    if(step_inv) step_port->FIOSET= bits; else step_port->FIOCLR= bits;
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
        int32_t now[k_max_actuators];
        for (uint8_t m = 0; m < num_motors; m++) now[m]= motor[m]->get_current_step();
        latch_.take(now, num_motors);
    }
    const Limit &l= limits[limit_idx];
    int32_t into= motor[l.motor]->get_current_step() - latch_.steps[l.motor];
    if(!l.at_end) {
        into= abs(into);
    } else if(!l.at_max) {
        into= -into;
    }

    if(into < limit_hysteresis)
        return state_;

    latch_.trigger();
    limit_tripped= true;
    machine_task.halt(HARD_LIMIT, "hard limit");
    return state_;
}

void StepTicker::brake(bool may_resume, float scale)
{
    if(state_ == BRAKING || state_ == HELD) {
        if(!may_resume) resumable_= false;
        if(scale > brake_scale) brake_scale= scale;
        return;
    }
    if(state_ != MOVING) return;

    resumable_= may_resume;
    brake_scale= scale;
    state_= BRAKING;
    braking_written= false;
    defer_wake();
}

static float path_per_mm(const Block *b)
{
    return b->millimeters > 0.0F ? (float)b->steps_event_count() / b->millimeters : 0.0F;
}

void StepTicker::start_brake()
{
    if(last_interval == 0 || path_decel <= 0 || current_block == nullptr) {
        return;
    }
    braking_written= true;

    float v= timer_hz / (float)last_interval;
    brake_v2= v * v;
    brake_c= (float)last_interval;   // what the Newton step refines from
    brake_dv2= 2.0F * (float)path_decel;
    brake_per_mm= path_per_mm(current_block);
    // the brake is the profile's leg to rest: its deceleration follows 16 t^2 (1 - t)^2 of the
    // peak over its time T = v / mean, so it starts and ends without a jerk step
    float T= v * StepCompress::k_peak_over_mean / ((float)path_decel * brake_scale) * timer_hz;
    brake_t= 0.0F;
    brake_inv_T= T > 1.0F ? 1.0F / T : 1.0F;
}

// a brake that runs into the next block keeps its physical deceleration: the speed and the
// interval are in path steps, and the next block's path step may be another length
void StepTicker::rescale_brake()
{
    float k= path_per_mm(current_block);
    if(k > 0.0F && brake_per_mm > 0.0F) {
        float r= k / brake_per_mm;
        brake_v2*= r * r;
        brake_c/= r;
        brake_per_mm= k;
    }
    brake_dv2= 2.0F * (float)path_decel;
}

void StepTicker::hold(bool on)
{
    paused_= on;
    if(on) {
        brake(true, 1.0F);
    }else{
        defer_wake();
    }
}

void StepTicker::stop_jog()
{
    brake(false, jog_limit);
}

// the closed-loop steppers hold the last step: a watch hit needs no brake, and a search is not resumed
void StepTicker::stand()
{
    if(state_ != MOVING && state_ != BRAKING) return;
    for (uint8_t m = 0; m < num_motors; m++) motor[m]->stop_moving();
    resumable_= false;
    current_tick= 0;
    state_= HELD;
    defer_wake();
}


uint32_t StepTicker::held_path() const
{
    return state_ == HELD ? mix.pos : 0;
}

void StepTicker::release(bool resume)
{
    if(state_ != HELD) {
        return;
    }

    resumable_= true;
    braking_written= false;

    held_block= resume ? current_block : nullptr;
    current_block= nullptr;
    if(!resume) {
        mix.reset();
    }

    state_= IDLE;
}

StepTicker::Motion StepTicker::check_watch()
{
    bool asserted= watch->inputs.any();
    if(asserted && !watch->seen) watch->witnessed= watch->witness.any();

    int32_t now[k_max_actuators];
    for (uint8_t m = 0; m < num_motors; m++) now[m]= motor[m]->get_current_step();
    if(!watch->sample(asserted, now, num_motors, latch_)) return state_;

    if(!watch->observe) stand();
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

    // a halt clears the ticker from any state; the unlock's flush still needs the queue dropped here
    if(machine_task.is_halted()) {
        stream.clear();
        current_block= nullptr;
        held_block= nullptr;
        mix.reset();
        current_tick= 0;
        state_= IDLE;
        THECONVEYOR.drop_queue();
        return 0;
    }

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
        if(current_block != nullptr) THECONVEYOR.block_finished();
        current_block= THECONVEYOR.take_block(stream.mark());
        path_decel= stream.mark_decel();
        stream.take_mark();
        if(!start_next_block()) return 0;

        if(motion == BRAKING && braking_written) rescale_brake();
        if(motion != BRAKING) {
            motion= MOVING;
            state_= MOVING;
        }

        if(dir_lead != 0) return dir_lead;
    }

    if(motion == IDLE) {
        if(current_block == nullptr) return 0;
        motion= MOVING;
        state_= MOVING;
    }

    // 0 is the stream saying it has nothing: a real interval is never 0, take() floors it at
    // 1. The mark case was handled above, so this is the ring having run dry.
    uint32_t ticks= stream.take();

    if(ticks == 0) {
        if(mix.owing()) pulse(nullptr);
        if(current_block != nullptr && played_out()) end_block(motion);
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

    if(dir_lead != 0) {
        if(ticks > dir_lead + 1) ticks-= dir_lead;
        dir_lead= 0;
    }

    // A brake takes its step from the stream as usual, so the path and the blocks keep their
    // bookkeeping; only the interval is its own. v^2 reaching zero is the stand.
    if(motion == BRAKING && braking_written) {
        brake_t+= (float)ticks;
        float tau= brake_t * brake_inv_T;
        float w= tau * (1.0F - tau);
        brake_v2-= brake_dv2 * brake_scale * 16.0F * w * w;
        if(brake_v2 <= 0.0F || tau >= 1.0F) {
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

inline void StepTicker::pulse(StepMix::Player *p)
{
    uint32_t fire= 0;
    for (uint8_t m = 0; m < num_motors; m++) {
        uint8_t does= mix.motor(p, m);
        if(does == 0) continue;
        if(does == StepMix::TURNED) {
            motor[m]->set_direction((mix.pin_dirs >> m) & 1);
            continue;
        }
        fire|= step_bit[m];
        if(!motor[m]->count_step()) mix.drop(m);   // its moving flag was cleared from outside
        if(!mix.busy(m)) motor[m]->stop_moving();
    }

    // every motor due steps in the one write; TIMER1 takes the pins down again after the pulse
    if(fire != 0) {
        if(step_inv) step_port->FIOCLR= fire; else step_port->FIOSET= fire;
        unstep|= fire;
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }
}

void StepTicker::end_block(Motion motion)
{
    // steps that cancelled across a corner end a motor without a pulse
    for (uint8_t m = 0; m < num_motors; m++) motor[m]->stop_moving();
    current_tick= 0;
    if(current_block != nullptr) THECONVEYOR.block_finished();
    current_block= nullptr;
    state_= motion == BRAKING ? HELD : IDLE;
    defer_wake();
}

inline uint32_t StepTicker::issue_step(uint32_t ticks, Motion motion)
{
    StepMix::Player *p= mix.tick();
    uint32_t w= current_block != nullptr ? current_block->blend_out : 0;
    if(mix.wants_next(w)) {
        mix.open(*THECONVEYOR.next_block(), w);
        for (uint8_t m = 0; m < num_motors; m++) {
            if(mix.other.left[m] != 0) motor[m]->start_moving();
        }
    }
    pulse(p);

    current_tick++;

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

    poll_left-= (int32_t)ticks;
    if(poll_left <= 0) {
        poll_left= poll_ticks;
        if(watch != nullptr && !watch->hit) {
            motion= check_watch();
        }
        if(n_limits != 0 && !limit_tripped) {
            motion= check_limits();
        }
    }

    if(played_out()) end_block(motion);

    return stream.empty() ? 0 : ticks;
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool resume= current_block == held_block;
    held_block= nullptr;

    if(!resume && mix.two && mix.first_half) {
        mix.swap_at_mark();
    } else if(!resume) {
        mix.n= num_motors;
        mix.start(*current_block);
        uint8_t pins= 0;
        for (uint8_t m = 0; m < num_motors; m++) {
            if(mix.lead.left[m] != 0) {
                bool dir= (current_block->direction_bits >> m) & 1;
                if(motor[m]->which_direction() != dir) dir_lead= dir_lead_ticks;
                motor[m]->set_direction(dir);
            }
            if(motor[m]->which_direction()) pins|= 1 << m;
        }
        mix.pin_dirs= pins;
    }

    bool ok= false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if(!mix.busy(m)) continue;
        ok= true;
        motor[m]->start_moving();
    }
    current_tick= 0;

    if(ok) {
        return true;
    }

    // a block with no steps for any motor
    THECONVEYOR.block_finished();
    current_block= nullptr;
    return false;
}


// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
