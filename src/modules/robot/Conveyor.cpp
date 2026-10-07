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

void Conveyor::resume_held()
{
    StepTicker &ticker= THEKERNEL->step_ticker;
    if(ticker.motion() != StepTicker::HELD || queue.isr_tail_i == queue.head_i) return;

    ticker.steps().clear();
    StepCompress::rewind();
    fed_i= queue.isr_tail_i;
    swept= false;
    fed_from= ticker.held_path();
    fed_steps= 0;
    fed_started= false;
    entry2= 0.0F;
}

static float step_length(const Block *b)
{
    return b->millimeters / (float)b->steps_event_count();
}

// A block's acceleration against speed^2, linear between knots, constant past the last; D at
// each knot is the mm from rest.
struct AccelCurve {
    float w[Robot::k_max_knots], a[Robot::k_max_knots], D[Robot::k_max_knots];
    uint8_t n;

    void of(const Block *b)
    {
        n= THEROBOT.accel_knots(*b, w, a);
        D[0]= 0.0F;
        for (uint8_t k = 1; k < n; k++)
            D[k]= D[k - 1] + span(a[k - 1], slope(k - 1), w[k] - w[k - 1]);
    }

    float slope(uint8_t k) const
    {
        return k + 1 < n ? (a[k + 1] - a[k]) / (w[k + 1] - w[k]) : 0.0F;
    }

    // mm for dw of speed^2 on a piece where a = a0 + be (w - w0)
    static float span(float a0, float be, float dw)
    {
        return be == 0.0F ? dw / (2.0F * a0) : log1pf(be * dw / a0) / (2.0F * be);
    }

    // speed^2 gained over x mm on such a piece
    static float unspan(float a0, float be, float x)
    {
        return be == 0.0F ? 2.0F * a0 * x : a0 * expm1f(2.0F * be * x) / be;
    }

    // mm from rest to speed^2 v2
    float dist(float v2) const
    {
        uint8_t k= n - 1;
        while(k > 0 && w[k] > v2)
            k--;
        return D[k] + span(a[k], slope(k), v2 - w[k]);
    }

    // speed^2 after x mm from rest
    float speed2(float x) const
    {
        if(!(x > 0.0F))
            return 0.0F;

        uint8_t k= n - 1;
        while(k > 0 && D[k] > x)
            k--;
        return w[k] + unspan(a[k], slope(k), x - D[k]);
    }
};

// the curves of the blocks one feed pass plans
struct CurveCache {
    static const uint8_t k_size= 3;
    AccelCurve curve[k_size];
    const Block *block[k_size];
    uint8_t next;

    void clear()
    {
        for (uint8_t i = 0; i < k_size; i++)
            block[i]= nullptr;
    }

    const AccelCurve &of(const Block *b)
    {
        for (uint8_t i = 0; i < k_size; i++) {
            if(block[i] == b)
                return curve[i];
        }
        uint8_t i= next;
        next= (uint8_t)((next + 1) % k_size);
        block[i]= b;
        curve[i].of(b);
        return curve[i];
    }
};
static CurveCache curves;

// plans the speeds for the rest of the block, from step at0 to its end, on the block's a(v)
static void solve(const Block *b, float entry2, float &exit2, StepCompress::Span &s)
{
    const AccelCurve &c= curves.of(b);
    uint32_t total= s.whole - s.at0;
    float d= s.dist(0, total);
    float per_mm= (float)total / d;
    float nominal2= b->nominal_speed * b->nominal_speed;
    if(entry2 > nominal2)
        entry2= nominal2;
    if(exit2 > nominal2)
        exit2= nominal2;

    float de= c.dist(entry2), dx= c.dist(exit2);
    if(dx > de + d) {
        dx= de + d;
        exit2= c.speed2(dx);
    }
    // within rounding the asked exit stands: a block ending at rest ends at zero
    if(dx < de - d - 1e-3F * de) {
        dx= de - d;
        exit2= c.speed2(dx);
    }

    float dp= 0.5F * (d + de + dx);
    float peak2= c.speed2(dp);
    if(peak2 > nominal2) {
        peak2= nominal2;
        dp= c.dist(peak2);
    }
    if(dp < de) {
        dp= de;
        peak2= entry2;
    }
    if(dp < dx) {
        dp= dx;
        peak2= exit2;
    }

    uint32_t up= (uint32_t)((dp - de) * per_mm + 0.5F);
    uint32_t down= (uint32_t)((dp - dx) * per_mm + 0.5F);
    if(up > total)
        up= total;
    if(down > total - up)
        down= total - up;
    s.up= up;
    s.down= down;
    s.flat= total - up - down;

    // plateau
    float most= dp;
    if(up != 0 && de + (float)up / per_mm < most)
        most= de + (float)up / per_mm;
    if(down != 0 && dx + (float)down / per_mm < most)
        most= dx + (float)down / per_mm;
    if(most < dp)
        peak2= c.speed2(most);

    s.v_entry= sqrtf(entry2);
    s.v_flat= sqrtf(peak2);
    s.v_exit= sqrtf(exit2);
    s.accel= THEROBOT.path_accel(*b, s.v_flat);
    s.accel_in= THEROBOT.path_accel(*b, s.v_entry);
    s.accel_out= THEROBOT.path_accel(*b, s.v_exit);
}

// the spans solve() worked out lately
struct PlanMemo {
    static const uint8_t k_size= 8;
    struct Entry {
        bool used;
        uint32_t serial, at0, stamp;
        float entry2, asked2, exit2;
        uint32_t up, flat, down;
        float v_entry, v_flat, v_exit, accel, accel_in, accel_out;
    } e[k_size];
    uint8_t next;
};
static PlanMemo memo;

static void plan(const Block *b, float entry2, float &exit2, StepCompress::Span &s)
{
    uint32_t stamp= THEROBOT.accel_stamp;
    for (uint8_t k = 0; k < PlanMemo::k_size; k++) {
        const PlanMemo::Entry &m= memo.e[k];
        if(!m.used || m.serial != b->serial || m.at0 != s.at0 || m.stamp != stamp
           || m.entry2 != entry2 || m.asked2 != exit2)
            continue;

        s.up= m.up;
        s.flat= m.flat;
        s.down= m.down;
        s.v_entry= m.v_entry;
        s.v_flat= m.v_flat;
        s.v_exit= m.v_exit;
        s.accel= m.accel;
        s.accel_in= m.accel_in;
        s.accel_out= m.accel_out;
        exit2= m.exit2;
        return;
    }
    float asked2= exit2;
    solve(b, entry2, exit2, s);
    PlanMemo::Entry &m= memo.e[memo.next];
    memo.next= (uint8_t)((memo.next + 1) % PlanMemo::k_size);
    m= {true, b->serial, s.at0, stamp, entry2, asked2, exit2, s.up, s.flat, s.down,
        s.v_entry, s.v_flat, s.v_exit, s.accel, s.accel_in, s.accel_out};
}

bool Conveyor::span_of(unsigned int i, uint32_t from, float entry2, float &exit2,
                       StepCompress::Span &s) const
{
    const Block *b= queue.item_ref(i);
    uint32_t whole= b->steps_event_count();
    if(whole <= from || b->millimeters <= 0.0F) return false;
    s.ds= step_length(b);
    s.v_max_entry= b->max_entry_speed;

    s.at0= from;
    s.whole= whole;

    plan(b, entry2, exit2, s);
    return true;
}

// reverse pass, in squared speeds; it stops at the first old block whose limit is unchanged
void Conveyor::sweep()
{
    float rest= THEKERNEL->planner.rest_speed();
    float next2= rest * rest;
    unsigned int end= end_i();
    if(end == fed_i)
        return;

    uint32_t stamp= THEROBOT.accel_stamp;
    bool again= swept && stamp == swept_stamp;
    if(again && end == swept_end)
        return;

    unsigned int last= queue.prev(swept_end);
    bool old= false;
    unsigned int i= queue.prev(end);
    while(i != fed_i) {
        const Block *b= queue.item_ref(i);
        float l2= b->max_entry_speed * b->max_entry_speed;
        if(l2 > next2) {
            const AccelCurve &c= curves.of(b);
            float reach2= c.speed2(c.dist(next2) + b->millimeters);
            if(reach2 < l2)
                l2= reach2;
        }
        if(i == last)
            old= true;
        if(again && old && limit2[i] == l2)
            break;

        limit2[i]= l2;
        next2= l2;
        i= queue.prev(i);
    }
    swept= true;
    swept_end= end;
    swept_stamp= stamp;
}

void Conveyor::feed_stream()
{
    StepTicker &ticker= THEKERNEL->step_ticker;

    unsigned int end= end_i();
    if(flush || fed_i == end) {
        return;
    }
    curves.clear();
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
            // replan the unwritten plateau when the exit speed has risen
            float nominal2= b->nominal_speed * b->nominal_speed;
            if(exit2 > nominal2) exit2= nominal2;
            if(exit2 > fed_exit2) {
                fed_from+= fed_steps;
                fed_steps= 0;
                fed_exit2= exit2;
                fed.at0= fed_from;
                plan(b, fed.v_flat * fed.v_flat, fed_exit2, fed);
            }
        }
        const StepCompress::Span &s= fed;
        uint32_t total= s.up + s.flat + s.down;

        // enough queued to brake from the block's speed with margin: a late machine task then
        // costs a controlled stop at worst, never a stand at speed
        float hz= ticker.rate();
        float ahead= k_feed_ahead_ms / 1000.0F;
        if(s.accel > 0.0F) {
            ahead+= 1.5F * s.v_flat / (s.accel * StepCompress::k_peak_over_mean);
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
            int32_t decel= (int32_t)(s.accel * StepCompress::k_peak_over_mean / s.ds);
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
            fed_steps= StepCompress::ramp(ticker.steps(), t, s, up, fed_steps);
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
            fed_steps+= StepCompress::plateau(ticker.steps(), s.v_flat, s, steps);
        }

        if(fed_steps >= plateau_end && fed_steps < total) {
            StepCompress::Target t= StepCompress::target(StepCompress::DECEL, fed_steps, s, next);
            uint32_t into= StepCompress::ramp(ticker.steps(), t, s, s.down,
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
        swept= false;
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
        swept= false;
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
    // moves a running code queues itself, like the clamp homing or a laser sweep, have mark 0:
    // they pass, or the code would wait on them until the next step
    while(fence.on && fence.at != queue.head_i && queue.item_ref(fence.at)->mark == 0) {
        place_fence(queue.next(fence.at));
    }
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
    if(i != queue.head_i)
        i= past_line(i);
    place_fence(i);
    return true;
}

// the blocks from i on are unfed: their corner at the fence can still become a stop
void Conveyor::place_fence(unsigned int i)
{
    unsigned int behind= (queue.head_i + BLOCK_QUEUE_LENGTH - i) % BLOCK_QUEUE_LENGTH;
    fence.edge= queued - behind;
    fence.edge_mark= i == queue.isr_tail_i ? executed : queue.item_ref(queue.prev(i))->mark;
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

    queue.head_ref()->serial= queued;
    queue.produce_head();
    queued++;

    // not sure if this is the correct place but we need to turn on the motors if they were not already on
    THEROBOT.enable_motors(true);
}


void Conveyor::wake_on_block(TaskHandle_t t)
{
    server= t;
    defer_wake_to(WAKE_MACHINE, t, k_notify_index);
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
