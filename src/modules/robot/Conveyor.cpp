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

void Conveyor::rewind_feed()
{
    THEKERNEL->step_ticker.steps().clear();
    StepCompress::rewind();
    for (unsigned int i = queue.isr_tail_i; i != queue.head_i; i= queue.next(i)) {
        queue.item_ref(i)->is_ticking= false;
    }
    fed_i= queue.isr_tail_i;
    fed_steps= 0;
}

// false for a block with nothing left to step
static bool span_of(const Block *b, StepCompress::Span &s)
{
    uint32_t whole= b->steps_event_count();
    uint32_t total= whole > b->resume_at ? whole - b->resume_at : 0;
    if(total == 0 || b->millimeters <= 0.0F) return false;
    s.ds= b->millimeters / (float)whole;
    s.up= b->ramp.accel_steps;
    s.down= b->ramp.decel_steps;
    s.flat= total - s.up - s.down;
    s.v_entry= b->ramp.entry_rate * s.ds;
    s.v_flat= b->ramp.plateau_rate * s.ds;
    s.v_exit= b->ramp.exit_rate * s.ds;
    s.v_max_entry= b->max_entry_speed;
    s.accel= b->acceleration;
    return true;
}

void Conveyor::feed_stream()
{
    StepTicker &ticker= THEKERNEL->step_ticker;

    if(flush) {
        return;
    }

    while(fed_i != queue.head_i) {
        Block *b= queue.item_ref(fed_i);
        if(!b->is_ready || b->locked) {
            break;
        }

        StepCompress::Span s;
        if(!span_of(b, s)) {
            fed_i= queue.next(fed_i);
            fed_steps= 0;
            continue;
        }
        uint32_t total= s.up + s.flat + s.down;

        // enough queued to brake from the block's speed with margin: a late machine task then
        // costs a controlled stop at worst, never a stand at speed
        if(fed_steps == 0 && !ticker.steps().empty()) {
            float hz= ticker.rate();
            float ahead= k_feed_ahead_ms / 1000.0F;
            if(b->acceleration > 0.0F) ahead+= 1.5F * s.v_flat / (b->acceleration * StepCompress::k_peak_over_mean);
            if(hz > 0.0F && ticker.steps().ticks_queued() >= (uint32_t)(hz * ahead)) {
                break;
            }
        }

        if(ticker.steps().full()) {
            break;
        }

        if(fed_steps == 0) {
            // the planner's acceleration is the mean; a brake decelerates at the profile's peak
            int32_t decel= (int32_t)(b->acceleration * StepCompress::k_peak_over_mean / s.ds);
            if(decel < 1) decel= 1;
            if(!ticker.steps().push_mark(fed_i, decel)) {
                break;
            }
            b->is_ticking= true;
        }

        auto next= [&](uint8_t j, StepCompress::Span &out) {
            unsigned int i= fed_i;
            for (uint8_t n = 0; n < j; n++) {
                i= queue.next(i);
                if(i == queue.head_i) return false;
            }
            const Block *p= queue.item_ref(i);
            return p->is_ready && !p->locked && span_of(p, out);
        };

        uint32_t up= s.up;
        uint32_t plateau_end= s.up + s.flat;

        if(fed_steps < up) {
            StepCompress::Target t= StepCompress::target(StepCompress::ACCEL, fed_steps, s, next);
            fed_steps= StepCompress::ramp(ticker.steps(), t, s.ds, up, fed_steps);
            if(fed_steps < up) {
                break;                   // the ring filled inside the ramp
            }
        }

        if(fed_steps >= up && fed_steps < plateau_end) {
            fed_steps+= StepCompress::plateau(ticker.steps(), s.v_flat, s.ds, plateau_end - fed_steps);
        }

        if(fed_steps >= plateau_end && fed_steps < total) {
            StepCompress::Target t= StepCompress::target(StepCompress::DECEL, fed_steps, s, next);
            uint32_t into= StepCompress::ramp(ticker.steps(), t, s.ds, s.down, fed_steps - plateau_end);
            fed_steps= plateau_end + into;
        }

        if(fed_steps < total) {
            break;
        }

        fed_i= queue.next(fed_i);
        fed_steps= 0;
    }
}

void Conveyor::service()
{
    // running is false while the main task drains: then every block goes to the ticker at once
    feed_stream();

    collect();

    // the actions went with the blocks they were written after. the moves went with them too,
    // so the planner is now ahead of the machine and has to be pulled back
    if(flush && queue.is_empty()) {
        pending_actions.clear();
        flush= false;
        fed_i= queue.head_i;
        fed_steps= 0;
        StepCompress::rewind();
        // the dropped blocks never went through block_finished: everything queued is over now,
        // or a later refusal waits on a mark that is never reached
        finished= queued;
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
        for (unsigned int i = queue.isr_tail_i; i != queue.head_i; i= queue.next(i)) {
            queue.item_ref(i)->is_ticking= false;
        }
        fed_i= queue.isr_tail_i;
        fed_steps= 0;
        StepCompress::rewind();
    }

    // an action handler reaches the conveyor again through its own calls; it must not recurse here
    if(in_actions != nullptr) return;
    in_actions= xTaskGetCurrentTaskHandle();
    pending_actions.run_upto(finished);
    in_actions= nullptr;
}

// see if we are idle
// this checks the block queue is empty, and that the step queue is empty and
unsigned int Conveyor::running_line() const
{
    const Block *block= THEKERNEL->step_ticker.get_current_block();
    if(block != nullptr && block->is_ready) return block->line;
    return 0;
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
    current_feedrate= b->nominal_speed;
    return b;
}

void Conveyor::block_finished()
{
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
