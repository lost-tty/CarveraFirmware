/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "nuts_bolts.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "Watchdog.h"
#include "Timer.h" // mbed.h lib
#include "wait_api.h" // mbed.h lib
#include "Block.h"
#include "Conveyor.h"
#include "mri.h"
#include "checksumm.h"
#include "Logging.h"
#include "Robot.h"
#include "MachineTask.h"
#include "StepperMotor.h"
#include "libs/StepCompress.h"
#include "libs/DeferredWake.h"

#include <functional>

#include "mbed.h"



/*
 * The conveyor holds the queue of blocks, takes care of creating them, and starting the executing chain of blocks
 *
 * The Queue is implemented as a ringbuffer- with a twist
 *
 * Since delete() is not thread-safe, we must marshall deletable items out of ISR context
 *
 * To do this, we have implmented a *double* ringbuffer- two ringbuffers sharing the same ring, and one index pointer
 *
 * as in regular ringbuffers, HEAD always points to a clean, free block. We are free to prepare it as we see fit, at our leisure.
 * When the block is fully prepared, we increment the head pointer, and from that point we must not touch it anymore.
 *
 * also, as in regular ringbuffers, we can 'use' the TAIL block, and increment tail pointer when we're finished with it
 *
 * Both of these are implemented here- see queue_head_block() (where head is pushed) and service() (where tail is consumed)
 *
 * The double ring is implemented by adding a third index pointer that lives in between head and tail. We call it isr_tail_i.
 *
 * in ISR context, we use HEAD as the head pointer, and isr_tail_i as the tail pointer.
 * As HEAD increments, ISR context can consume the new blocks which appear, and when we're finished with a block, we increment isr_tail_i to signal that they're finished, and ready to be cleaned
 *
 * in IDLE context, we use isr_tail_i as the head pointer, and TAIL as the tail pointer.
 * When isr_tail_i != tail, we clean up the tail block (performing ISR-unsafe delete operations) and consume it (increment tail pointer), returning it to the pool of clean, unused blocks which HEAD is allowed to prepare for queueing
 *
 * Thus, our two ringbuffers exist sharing the one ring of blocks, and we safely marshall used blocks from ISR context to IDLE context for safe cleanup.
 */


void Conveyor::init()
{
    running = false;
    flush = false;
}

void Conveyor::on_module_loaded()
{
    initialized= true;
}

// we allocate the queue here after config is completed so we do not run out of memory during config
void Conveyor::start(uint8_t n_actuators)
{
    if(!initialized) {
        printk("FATAL: conveyor not configured, the machine will not move\n");
        return;
    }
    Block::init(n_actuators);

    running = true;
}

void Conveyor::cleanup()
{
    flush_queue();
}

bool Conveyor::can_blend(uint32_t w) const
{
    if(queue.is_empty() || fed_i == queue.head_i) return false;
    if(fence.on && fence.at == queue.head_i)
        return false;

    if(pending_actions.waiting_after(queued)) return false;
    if(fed_i != queue.prev(queue.head_i)) return true;
    // a block being fed can still blend while in its plateau, with the window not yet written
    if(fed_started && (fed_steps < fed.up || fed_steps >= fed.up + fed.flat)) return false;
    return fed_from + fed_steps + w <= queue.item_ref(fed_i)->steps_event_count();
}

void Conveyor::resume_held()
{
    StepTicker &ticker= THEKERNEL->step_ticker;
    if(ticker.motion() != StepTicker::HELD || queue.isr_tail_i == queue.head_i) return;

    ticker.steps().clear();
    StepCompress::rewind();
    fed_i= queue.isr_tail_i;
    fed_from= ticker.held_path();
    fed_steps= 0;
    fed_started= false;
    entry2= 0.0F;
}

static float step_length(const Block *b)
{
    return b->millimeters / (float)b->steps_event_count();
}

// plans the speeds for the rest of the block, from step at0 to its end
static void plan(const Block *b, float entry2, float &exit2, StepCompress::Span &s)
{
    uint32_t total= s.whole - s.at0;
    float a2= 2.0F * b->acceleration;
    float d= s.dist(0, total);
    float per_step= d / (float)total;
    float nominal2= b->nominal_speed * b->nominal_speed;
    if(entry2 > nominal2) entry2= nominal2;
    if(exit2 > nominal2) exit2= nominal2;
    if(exit2 > entry2 + a2 * d) exit2= entry2 + a2 * d;
    // within rounding the asked exit stands: a block ending at rest ends at zero
    if(exit2 < entry2 - a2 * d - 1e-3F * entry2) exit2= entry2 - a2 * d;

    float peak2= 0.5F * (a2 * d + entry2 + exit2);
    if(peak2 > nominal2) peak2= nominal2;
    if(peak2 < entry2) peak2= entry2;
    if(peak2 < exit2) peak2= exit2;
    uint32_t up= (uint32_t)((peak2 - entry2) / (a2 * per_step) + 0.5F);
    uint32_t down= (uint32_t)((peak2 - exit2) / (a2 * per_step) + 0.5F);
    if(up > total) up= total;
    if(down > total - up) down= total - up;
    s.up= up;
    s.down= down;
    s.flat= total - up - down;

    // plateau
    float reach2= entry2 + a2 * (float)up * per_step;
    if(up != 0 && reach2 < peak2) peak2= reach2;
    reach2= exit2 + a2 * (float)down * per_step;
    if(down != 0 && reach2 < peak2) peak2= reach2;

    s.v_entry= sqrtf(entry2);
    s.v_flat= sqrtf(peak2);
    s.v_exit= sqrtf(exit2);
}

bool Conveyor::span_of(unsigned int i, uint32_t from, float entry2, float &exit2,
                       StepCompress::Span &s) const
{
    const Block *b= queue.item_ref(i);
    uint32_t whole= b->steps_event_count();
    if(whole <= from || b->millimeters <= 0.0F) return false;
    s.ds= step_length(b);
    s.v_max_entry= b->max_entry_speed;
    s.accel= b->acceleration;

    s.at0= from;
    s.whole= whole;
    s.in= 0;
    // the previous block may already be freed: the rest of its corner then uses this step length
    if(b->blend_in != 0 && i != queue.tail_i) {
        s.in= b->blend_in;
        s.ds_in= 0.5F * (step_length(queue.item_ref(queue.prev(i))) + s.ds);
    }
    window_out(i, s);

    plan(b, entry2, exit2, s);
    return true;
}

void Conveyor::window_out(unsigned int i, StepCompress::Span &s) const
{
    s.out= queue.item_ref(i)->blend_out;
    if(s.out != 0) s.ds_out= 0.5F * (s.ds + step_length(queue.item_ref(queue.next(i))));
}

// reverse pass, in squared speeds
void Conveyor::sweep()
{
    float rest= THEKERNEL->planner.rest_speed();
    float next2= rest * rest;
    unsigned int i= end_i();
    while(i != fed_i) {
        i= queue.prev(i);
        if(i == fed_i) break;
        const Block *b= queue.item_ref(i);
        float l2= b->max_entry_speed * b->max_entry_speed;
        float reach2= next2 + 2.0F * b->acceleration * b->millimeters;
        if(reach2 < l2) l2= reach2;
        limit2[i]= l2;
        next2= l2;
    }
}

void Conveyor::feed_stream()
{
    StepTicker &ticker= THEKERNEL->step_ticker;

    unsigned int end= end_i();
    if(flush || fed_i == end) {
        return;
    }
    sweep();
    float rest= THEKERNEL->planner.rest_speed();
    float rest2= rest * rest;

    while(fed_i != end) {
        Block *b= queue.item_ref(fed_i);
        if(!b->is_ready) {
            break;
        }

        unsigned int n= queue.next(fed_i);
        float exit2= n == end ? rest2 : limit2[n];
        if(!fed_started) {
            fed_exit2= exit2;
            if(!span_of(fed_i, fed_from, entry2, fed_exit2, fed)) {
                fed_i= n;
                fed_from= 0;
                continue;
            }
        } else if(fed_steps >= fed.up && fed_steps < fed.up + fed.flat) {
            // replan the unwritten plateau when the exit speed has risen or a blend was added
            float nominal2= b->nominal_speed * b->nominal_speed;
            if(exit2 > nominal2) exit2= nominal2;
            if(exit2 > fed_exit2 || b->blend_out != fed.out) {
                fed_from+= fed_steps;
                fed_steps= 0;
                fed_exit2= exit2;
                fed.at0= fed_from;
                window_out(fed_i, fed);
                plan(b, fed.v_flat * fed.v_flat, fed_exit2, fed);
            }
        }
        const StepCompress::Span &s= fed;
        uint32_t total= s.up + s.flat + s.down;

        // enough queued to brake from the block's speed with margin: a late machine task then
        // costs a controlled stop at worst, never a stand at speed
        float hz= ticker.rate();
        float ahead= k_feed_ahead_ms / 1000.0F;
        if(b->acceleration > 0.0F) {
            ahead+= 1.5F * s.v_flat / (b->acceleration * StepCompress::k_peak_over_mean);
        }
        uint32_t margin= (uint32_t)(hz * ahead);
        if(!fed_started && ticker.steps().ticks_queued() >= margin) {
            break;
        }

        if(ticker.steps().full()) {
            break;
        }

        if(!fed_started) {
            // the planner's acceleration is the mean; a brake decelerates at the profile's peak
            int32_t decel= (int32_t)(b->acceleration * StepCompress::k_peak_over_mean / s.ds);
            if(decel < 1) decel= 1;
            if(!ticker.steps().push_mark(fed_i, decel)) {
                break;
            }
            fed_started= true;
        }

        unsigned int chain_i= fed_i;
        uint8_t chain_j= 0;
        float chain_exit2= fed_exit2;
        auto next= [&](uint8_t j, StepCompress::Span &out) {
            if(j != chain_j + 1) {
                chain_i= fed_i;
                chain_j= 0;
                chain_exit2= fed_exit2;
            }
            while(chain_j < j) {
                chain_i= queue.next(chain_i);
                if(chain_i == end)
                    return false;

                if(!queue.item_ref(chain_i)->is_ready) return false;
                unsigned int after= queue.next(chain_i);
                float exit2= after == end ? rest2 : limit2[after];
                if(!span_of(chain_i, 0, chain_exit2, exit2, out)) return false;
                chain_exit2= exit2;
                chain_j++;
            }
            return true;
        };

        uint32_t up= s.up;
        uint32_t plateau_end= s.up + s.flat;

        if(fed_steps < up) {
            StepCompress::Target t= StepCompress::target(StepCompress::ACCEL, fed_steps, s, next);
            fed_steps= StepCompress::ramp(ticker.steps(), t, s, 0, up, fed_steps);
            if(fed_steps < up) {
                break;                   // the ring filled inside the ramp
            }
        }

        if(fed_steps >= up && fed_steps < plateau_end) {
            uint32_t steps= plateau_end - fed_steps;
            float cap2= b->nominal_speed * b->nominal_speed;
            if(n != end) {
                float junction= queue.item_ref(n)->max_entry_speed;
                if(junction * junction < cap2) cap2= junction * junction;
            }
            // while the exit speed can still rise, write only the margin ahead
            bool open= n == end || fed_exit2 < cap2;
            uint32_t most= open ? margin : (uint32_t)(hz * k_written_ahead_s);
            uint32_t queued= ticker.steps().ticks_queued();
            if(queued >= most) {
                break;
            }
            float room= (float)(most - queued) * s.v_flat / (hz * s.ds);
            if(room < (float)steps) steps= (uint32_t)room + 1;
            fed_steps+= StepCompress::plateau(ticker.steps(), s.v_flat, s, fed_steps, steps);
        }

        if(fed_steps >= plateau_end && fed_steps < total) {
            StepCompress::Target t= StepCompress::target(StepCompress::DECEL, fed_steps, s, next);
            uint32_t into= StepCompress::ramp(ticker.steps(), t, s, plateau_end, s.down,
                                              fed_steps - plateau_end);
            fed_steps= plateau_end + into;
        }

        if(fed_steps < total) {
            break;
        }

        fed_i= queue.next(fed_i);
        fed_from= 0;
        fed_steps= 0;
        fed_started= false;
        entry2= fed_exit2;
    }
}

void Conveyor::service()
{
    apply_fence();
    // running is false while the main task drains: then every block goes to the ticker at once
    feed_stream();

    collect();

    // the actions went with the blocks they were written after. the moves went with them too,
    // so the planner is now ahead of the machine and has to be pulled back
    if(flush && queue.is_empty()) {
        pending_actions.clear();
        fence.on= false;
        flush= false;
        fed_i= queue.head_i;
        fed_from= 0;
        fed_steps= 0;
        fed_started= false;
        entry2= 0.0F;
        StepCompress::rewind();
        // the dropped blocks never went through block_finished: everything queued is over now,
        // or a later refusal waits on a mark that is never reached
        finished= queued;
        playing= 0;
        THEROBOT.reset_position_from_current_actuator_position();
    }
}

// a block the step ticker has finished with is only freed here
void Conveyor::collect()
{
    if (queue.tail_i == queue.isr_tail_i) return;

    if (queue.is_empty()) {
        __debugbreak();
        return;
    }

    Block* block = queue.tail_ref();
    block->clear();
    queue.consume_tail();

    // a halt has already stopped the outputs: an action now would switch one back on
    if(machine_task.is_halted()) {
        pending_actions.clear();
        fence.on= false;
        fed_i= queue.isr_tail_i;
        fed_from= 0;
        fed_steps= 0;
        fed_started= false;
        entry2= 0.0F;
        StepCompress::rewind();
    }

    run_actions();
}

void Conveyor::run_actions()
{
    // an action handler reaches the conveyor again through its own calls; it must not recurse here
    if(in_actions != nullptr) return;
    in_actions= xTaskGetCurrentTaskHandle();
    if(fence.on) {
        pending_actions.run_upto(finished, fence.edge, fence.edge_mark);
    } else {
        pending_actions.run_upto(finished);
    }
    in_actions= nullptr;
}

void Conveyor::ask_fence(Fence op)
{
    fence.op= op;
    fence.asked++;
}

Conveyor::Fenced Conveyor::fenced() const
{
    unsigned int i= fence.at;
    if(!fence.on || i == queue.head_i)
        return Fenced{false, 0};

    return Fenced{true, queue.item_ref(i)->mark};
}

unsigned int Conveyor::past_line(unsigned int i) const
{
    uint32_t mark= queue.item_ref(i)->mark;
    do {
        i= queue.next(i);
    } while(i != queue.head_i && queue.item_ref(i)->mark == mark);
    return i;
}

void Conveyor::apply_fence()
{
    if(fence.done == fence.asked)
        return;

    switch(fence.op) {
        case FENCE_LINE:
            if(!fence_after_playing())
                return;

            break;
        case FENCE_PASS:
            if(fence.on && fence.at != queue.head_i)
                place_fence(past_line(fence.at));

            break;
        case FENCE_LIFT:
            fence.on= false;
            break;
    }
    fence.done= fence.asked;
    run_actions();
}

// false while braking: a held block is written again on resume, an idle one is not written yet
bool Conveyor::fence_after_playing()
{
    StepTicker &ticker= THEKERNEL->step_ticker;
    StepTicker::Motion m= ticker.motion();
    bool unwritten= m == StepTicker::IDLE && fed_i == queue.isr_tail_i && !fed_started;
    if(m != StepTicker::HELD && !unwritten)
        return false;

    unsigned int i= queue.isr_tail_i;
    if(i != queue.head_i) {
        const Block *b= queue.item_ref(i);
        uint32_t at= m == StepTicker::HELD ? ticker.held_path() : fed_from;
        bool in_corner= at + b->blend_out > b->steps_event_count();
        i= past_line(i);
        // stopped in the corner into the next line, that line ends first
        if(in_corner && i != queue.head_i && queue.prev(i) == queue.isr_tail_i)
            i= past_line(i);
    }
    place_fence(i);
    return true;
}

// the blocks from i on are unfed: their corner at the fence can still become a stop
void Conveyor::place_fence(unsigned int i)
{
    unsigned int behind= (queue.head_i + BLOCK_QUEUE_LENGTH - i) % BLOCK_QUEUE_LENGTH;
    fence.edge= queued - behind;
    fence.edge_mark= i == queue.isr_tail_i ? executed : queue.item_ref(queue.prev(i))->mark;
    if(i != queue.head_i)
        queue.item_ref(i)->blend_in= 0;

    if(i != queue.isr_tail_i)
        queue.item_ref(queue.prev(i))->blend_out= 0;

    fence.at= i;
    fence.on= true;
}

void Conveyor::executed_unless_overtaken(uint32_t block, uint32_t mark)
{
    __disable_irq();
    if(finished == block) {
        executed= mark;
    }
    __enable_irq();
}

bool Conveyor::is_idle() const
{
    if(queue.is_empty()) {
        return !THEROBOT.any_motor_moving();
    }

    return false;
}

// Wait for the queue to be empty and for all the jobs to finish in step ticker
bool Conveyor::wait_for_idle(bool wait_for_motors)
{
    // draining from inside an action would undo its place in the path
    if(in_actions == xTaskGetCurrentTaskHandle()) return !machine_task.is_halted();

    bool halted= false;

    // wait for the job queue to empty, this means cycling everything on the block queue into the job queue
    // forcing them to be jobs
    running = false;
    while (!queue.is_empty()) {
        if(!wait_for_block(halted)) break;
    }

    if(wait_for_motors) {
        // now we wait for all motors to stop moving
        while(!halted && !is_idle()) {
            if(!wait_for_block(halted)) break;
        }
    }

    running = true;
    // returning now means that everything has totally finished
    return !halted;
}

/*
 * push the pre-prepared head block onto the queue
 */
void Conveyor::queue_head_block()
{
    // upstream caller will block on this until there is room in the queue
    bool halted= false;
    while (queue.is_full()) {
        if(!wait_for_block(halted)) break;
    }

    // nothing more goes on the queue once a halt or a stop is in: the block is dropped, and
    // produce_head would otherwise spin on a queue that is still full
    if(machine_task.interrupted()) {
        queue.head_ref()->clear();
        return;
    }

    queue.produce_head();
    queued++;

    // not sure if this is the correct place but we need to turn on the motors if they were not already on
    THEROBOT.enable_motors(true);
}


// called from step ticker ISR

// the step ticker is above configMAX_SYSCALL_INTERRUPT_PRIORITY, so it cannot notify directly
extern "C" void RIT_IRQHandler(void)
{
    THECONVEYOR.wake_server();
}

void Conveyor::wake_server()
{
    BaseType_t woken= pdFALSE;
    if(server != nullptr) vTaskNotifyGiveIndexedFromISR(server, k_notify_index, &woken);
    portYIELD_FROM_ISR(woken);
}

// called from step ticker ISR when block is finished, do not do anything slow here
Block *Conveyor::take_block(unsigned int i)
{
    queue.isr_tail_i= i;
    Block *b= queue.item_ref(i);
    playing= b->mark;
    current_feedrate= b->nominal_speed;
    return b;
}

void Conveyor::block_finished()
{
    const Block *b= queue.item_ref(queue.isr_tail_i);
    if(b->mark != 0) {
        executed= b->mark;
    }
    playing= 0;
    // we increment the isr_tail_i so we can get the next block
    queue.isr_tail_i= queue.next(queue.isr_tail_i);
    finished++;

    defer_wake();
}

// the step ticker only notifies when a block ends, so the timeout is there to re-check whether
// the motors have stopped
// a pending stop ends a wait the way a halt does: the waiter may be the very job that holds the
// machine task away from the loop that would run the stop
bool Conveyor::wait_for_block(bool &halted)
{
    if(machine_task.interrupted()) {
        halted= true;
        return false;
    }

    machine_task.tick();
    return true;
}

/*
    In most cases this will not totally flush the queue, as when streaming
    gcode there is one stalled waiting for space in the queue, in
    queue_head_block() so after this flush, once main_loop runs again one more
    gcode gets stuck in the queue, this is bad. Current work around is to call
    this when the queue in not full and streaming has stopped
*/
void Conveyor::force_queue()
{
    if(server != nullptr) xTaskNotifyGiveIndexed(server, k_notify_index);
}

bool Conveyor::hold_action(const McodeRegistry::Mcode *code, const Gcode &gcode)
{
    if(queued == finished) return false;
    return pending_actions.hold(code, gcode, queued);
}

// the blocks are dropped, not run, so there is nothing to wait for
void Conveyor::flush_queue()
{
    flush= true;
}

// from the step ISR, only while it stands: the queue is dropped in one move
void Conveyor::drop_queue()
{
    if(flush) {
        THEKERNEL->step_ticker.steps().clear();
        queue.isr_tail_i= queue.head_i;   // the feeder's own place is reset when the flush completes
    }
}

// Debug function
void Conveyor::dump_queue()
{
    for (unsigned int index = queue.tail_i, i = 0; true; index = queue.next(index), i++ ) {
        printk("block %03d > ", i);
        queue.item_ref(index)->debug();

        if (index == queue.head_i)
            break;
    }
}
