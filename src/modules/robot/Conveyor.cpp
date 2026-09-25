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
#include "Config.h"
#include "Logging.h"
#include "ConfigValue.h"
#include "Robot.h"
#include "MachineTask.h"
#include "StepperMotor.h"
#include "libs/StepCompress.h"
#include "libs/DeferredWake.h"

#include <functional>

#include "mbed.h"

#define brake_limit_checksum         CHECKSUM("brake_limit")

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
 * Both of these are implemented here- see queue_head_block() (where head is pushed) and on_idle() (where tail is consumed)
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
    brake_limit = THEKERNEL->config->value(brake_limit_checksum)->by_default(8.0F)->as_number();
    if(brake_limit < 1.0F) {
        printk("FATAL: brake_limit must be >= 1.0, got %f\n", brake_limit);
        return;
    }

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
    for (unsigned int i = queue.isr_tail_i; i != queue.head_i; i= queue.next(i)) {
        queue.item_ref(i)->is_ticking= false;
    }
    fed_i= queue.isr_tail_i;
    fed_steps= 0;
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

        unsigned int ahead= 0;
        for (unsigned int i= queue.isr_tail_i; i != fed_i; i= queue.next(i)) {
            ahead++;
        }
        if(ahead >= k_feed_ahead && queue.next(fed_i) != queue.head_i) {
            break;
        }

        uint32_t whole= b->steps_event_count();
        uint32_t total= whole > b->resume_at ? whole - b->resume_at : 0;
        if(total == 0) {
            fed_i= queue.next(fed_i);
            fed_steps= 0;
            continue;
        }

        if(ticker.steps().full()) {
            break;
        }

        uint32_t up= b->ramp.accel_steps;
        uint32_t down= b->ramp.decel_steps;
        if(up > total) up= total;
        if(down > total - up) down= total - up;
        uint32_t plateau_end= total - down;

        if(fed_steps == 0) {
            int32_t decel= 0;
            if(b->millimeters > 0.0F) {
                float per_mm= (float)whole / b->millimeters;
                decel= (int32_t)(b->acceleration * brake_limit * per_mm);
            }
            if(decel < 1) decel= 1;
            if(!ticker.steps().push_mark(fed_i, decel)) {
                break;
            }
            b->is_ticking= true;
        }

        if(fed_steps < up) {
            fed_steps= StepCompress::ramp(ticker.steps(), b->ramp.entry_rate,
                                          b->ramp.plateau_rate, up, fed_steps);
            if(fed_steps < up) {
                break;                   // the ring filled inside the ramp
            }
        }

        if(fed_steps >= up && fed_steps < plateau_end) {
            fed_steps+= StepCompress::plateau(ticker.steps(), b->ramp.plateau_rate,
                                              plateau_end - fed_steps);
        }

        if(fed_steps >= plateau_end && fed_steps < total) {
            uint32_t into= StepCompress::ramp(ticker.steps(), b->ramp.plateau_rate,
                                              b->ramp.exit_rate, down, fed_steps - plateau_end);
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

bool Conveyor::stop_soon()
{
    flush_queue();
    THEKERNEL->step_ticker.stop();   // brakes if it moves; the flush lands once it stands
    return wait_for_idle();
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
        queue.isr_tail_i= queue.head_i;
        fed_i= queue.head_i;
        fed_steps= 0;
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
