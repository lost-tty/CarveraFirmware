/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/Killable.h"
#include "BlockActions.h"
#include "BlockQueue.h"
#include "libs/StepCompress.h"

#include "FreeRTOS.h"
#include "task.h"

class Block;

#ifndef BLOCK_QUEUE_LENGTH
#define BLOCK_QUEUE_LENGTH 64
#endif

class Conveyor : public Module, public Killable
{
public:
    void init();
    void start(uint8_t n_actuators);

    void on_module_loaded(void);
    void kill() override {}
    void cleanup() override;

    void service();
    void wake_on_block(TaskHandle_t t);
    bool wait_for_idle(bool wait_for_motors=true); // false when a halt cut the wait short

    Block *take_block(unsigned int i);
    Block *next_block() { return queue.item_ref(queue.next(queue.isr_tail_i)); }
    // true if the newest queued block can still get a blend window of w steps at its end
    bool can_blend(uint32_t w) const;
    void block_finished();

    void flush_queue(void);

    void resume_held();

    enum Fence : uint8_t { FENCE_LINE, FENCE_PASS, FENCE_LIFT };
    void ask_fence(Fence op);   // done on the next pass of the machine task
    bool fence_settled() const { return fence.asked == fence.done; }
    bool fence_reached() const { return !fence.on || (int32_t)(finished - fence.edge) >= 0; }
    struct Fenced { bool any; uint32_t mark; };   // the first block behind the fence
    Fenced fenced() const;
    void drop_queue(void);   // ISR, while standing: the flushed queue goes in one move
    bool flushing() const { return flush; }
    void force_queue();   // a jog runs now, not after the pre-load wait

    // false: nothing is queued to wait on, so the caller runs it now
    bool hold_action(const McodeRegistry::Mcode *code, const Gcode &gcode);

    uint32_t queue_mark() const { return queued; }
    bool passed(uint32_t mark) const { return finished >= mark; }
    bool blocks_pending() const { return finished != queued; }
    unsigned int block_playing() const { return playing; }
    unsigned int last_executed() const { return executed; }
    void executed_unless_overtaken(uint32_t block, uint32_t mark);
    void clear_executed() { executed= 0; }

    bool is_idle() const;
    bool is_queue_empty() { return queue.is_empty(); };

    struct Planned { const Block *block; bool running; bool spent; bool free; };
    template<class F> void each_slot(F f) const
    {
        bool spent = true, free = false;
        unsigned i = queue.tail_i;
        do {
            if (i == queue.isr_tail_i) spent = false;
            if (i == queue.head_i) free = true;
            f(Planned{queue.item_ref(i), i == queue.isr_tail_i && !free, spent && !free, free});
            i = queue.next(i);
        } while (i != queue.tail_i);
    }

    float get_current_feedrate() const { return current_feedrate; }

    friend class Planner; // for queue

private:
    void dump_queue(void);
    bool is_queue_full() { return queue.is_full(); };
    void collect();
    void queue_head_block(void);

    static const uint32_t k_feed_ahead_ms= 60;
    static const uint32_t k_written_ahead_s= 20;   // far below the 171 s a 32-bit tick count covers
    void feed_stream();
    unsigned int end_i() const { return fence.on ? fence.at : queue.head_i; }
    unsigned int past_line(unsigned int i) const;
    void apply_fence();
    bool fence_after_playing();
    void place_fence(unsigned int i);
    void run_actions();

    static const UBaseType_t k_notify_index = 1;
    bool wait_for_block(bool &halted);

    using Queue_t = BlockQueue<BLOCK_QUEUE_LENGTH>;
    Queue_t queue; // Queue of Blocks

    volatile TaskHandle_t server{nullptr};   // the machine task, woken when a block ends

    BlockActions pending_actions;
    volatile TaskHandle_t in_actions{nullptr};   // the task inside an action handler, if any
    uint32_t queued{0};
    volatile uint32_t finished{0};
    volatile unsigned int playing{0}, executed{0};
    unsigned int fed_i{0};
    uint32_t fed_steps{0};
    bool fed_started{false};
    uint32_t fed_from{0};
    StepCompress::Span fed;
    float fed_exit2{0.0F};
    float entry2{0.0F};
    float limit2[BLOCK_QUEUE_LENGTH];
    struct FenceState {
        volatile Fence op{FENCE_LIFT};
        volatile uint8_t asked{0}, done{0};
        volatile bool on{false};
        volatile unsigned int at{0};
        uint32_t edge{0}, edge_mark{0};   // the last block before it, and its line
    };
    FenceState fence;
    void sweep();
    bool span_of(unsigned int i, uint32_t from, float entry2, float &exit2,
                 StepCompress::Span &s) const;
    void window_out(unsigned int i, StepCompress::Span &s) const;


    bool initialized{false};
    float current_feedrate{0}; // actual nominal feedrate that current block is running at in mm/sec

    // separate bytes so unlocked cross-task writes do not read-modify-write each other
    volatile bool running;
    volatile bool flush;
};
