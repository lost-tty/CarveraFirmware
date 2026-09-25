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

#include "FreeRTOS.h"
#include "task.h"

class Block;

extern "C" void RIT_IRQHandler(void);

class Conveyor : public Module, public Killable
{
public:
    void init();
    void start(uint8_t n_actuators);

    void on_module_loaded(void);
    void kill() override {}
    void cleanup() override;

    void service();
    void wake_on_block(TaskHandle_t t) { server= t; }
    bool wait_for_idle(bool wait_for_motors=true); // false when a halt cut the wait short
    bool stop_soon();

    bool get_next_block(Block **block);
    void block_finished();
    void wake_server();

    void flush_queue(void);
    void drop_queue(void);   // ISR, while standing: the flushed queue goes in one move
    bool flushing() const { return flush; }
    void force_queue();   // a jog runs now, not after the pre-load wait

    // false: nothing is queued to wait on, so the caller runs it now
    bool hold_action(const McodeRegistry::Mcode *code, const Gcode &gcode);

    uint32_t queue_mark() const { return queued; }
    bool passed(uint32_t mark) const { return finished >= mark; }

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

    unsigned int running_line() const;
    float get_current_feedrate() const { return current_feedrate; }

    friend class Planner; // for queue

private:
    void dump_queue(void);
    bool is_queue_full() { return queue.is_full(); };
    void check_queue(bool force= false);
    void collect();
    void queue_head_block(void);

    static const UBaseType_t k_notify_index = 1;
    bool wait_for_block(bool &halted);

    using Queue_t = BlockQueue<32>;
    Queue_t queue; // Queue of Blocks

    volatile TaskHandle_t server{nullptr};   // the machine task, woken when a block ends

    BlockActions pending_actions;
    volatile TaskHandle_t in_actions{nullptr};   // the task inside an action handler, if any
    uint32_t queued{0};
    volatile uint32_t finished{0};


    uint32_t queue_delay_time_ms;
    float current_feedrate{0}; // actual nominal feedrate that current block is running at in mm/sec

    // separate bytes so unlocked cross-task writes do not read-modify-write each other
    volatile bool running;
    volatile bool allow_fetch;
    volatile bool flush;
    volatile bool force_fetch;
};
