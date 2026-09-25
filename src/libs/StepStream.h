/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>

class StepStream
{
public:
    static const uint8_t k_add_shift = 8;
    static const uint16_t k_entries = 128;

    static const uint32_t k_mark = 0;

    struct Entry {
        uint32_t interval;
        uint32_t count;
        int32_t  add;
    };

    bool full() const { return next(head) == tail; }

    bool empty() const { return head == tail && left == 0; }
    uint16_t free_slots() const { return (uint16_t)(k_entries - 1 - used()); }
    uint16_t used() const { return (uint16_t)((head - tail + k_entries) % k_entries); }

    bool push(uint32_t interval, uint32_t count, int32_t add)
    {
        if(full()) {
            return false;
        }
        ring[head]= Entry{interval, count, add};
        head= next(head);
        return true;
    }

    bool at_mark() const { return !empty() && left == 0 && ring[tail].count == k_mark; }
    uint32_t mark() const { return ring[tail].interval; }
    int32_t mark_decel() const { return ring[tail].add; }
    void take_mark() { if(at_mark()) tail= next(tail); }

    bool push_mark(uint32_t move, int32_t decel) { return push(move, k_mark, decel); }

    uint32_t take()
    {
        while(left == 0) {
            if(empty() || ring[tail].count == k_mark) {
                return 0;
            }
            const Entry &e= ring[tail];
            scaled= (int64_t)e.interval << k_add_shift;
            add= e.add;
            left= e.count;
            tail= next(tail);
        }

        uint32_t ticks= (uint32_t)((scaled + (1 << (k_add_shift - 1))) >> k_add_shift);
        scaled+= add;
        left--;
        return ticks < 1 ? 1 : ticks;
    }

    void clear()
    {
        head= tail= 0;
        left= 0;
    }

    uint32_t queued_steps() const
    {
        uint32_t n= left;
        for (uint16_t i = tail; i != head; i= next(i)) {
            if(ring[i].count != k_mark) n+= ring[i].count;
        }
        return n;
    }

    void truncate(uint32_t keep)
    {
        if(left >= keep) {
            left= keep;
            head= tail;
            return;
        }

        uint32_t n= left;
        uint16_t i= tail;
        while(i != head) {
            if(ring[i].count != k_mark) {
                if(n + ring[i].count > keep) {
                    if(keep > n) {
                        ring[i].count= keep - n;
                        i= next(i);
                    }
                    break;
                }
                n+= ring[i].count;
            }
            i= next(i);
        }
        head= i;
    }

private:
    static uint16_t next(uint16_t i) { return (uint16_t)((i + 1) % k_entries); }

    Entry ring[k_entries];
    volatile uint16_t head{0};
    volatile uint16_t tail{0};

    int64_t scaled{0};
    int32_t add{0};
    uint32_t left{0};

};
