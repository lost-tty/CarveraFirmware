#pragma once

#include "Block.h"

#include <stdint.h>

// Across a blend window each path step advances one of two blocks, the one coming in with a
// weight rising from zero to one: each makes exactly its own steps, their sum is the parabola
struct StepMix {
    struct Player {
        uint32_t left[k_max_actuators];
        uint32_t acc[k_max_actuators];
        uint32_t share[k_max_actuators];
        uint32_t own, total;        // path steps of its block: made, in all
        uint16_t blend_in, blend_out;
        uint8_t dirs;
    };
    enum { FIRE= 1, TURNED= 2 };

    Player lead, other;             // the stream's block, and the other one of a window
    bool two{false};
    bool first_half{false};         // other is the block coming in
    uint32_t win_left{0};
    uint32_t pick{0}, weight{0}, gain{0};   // 0.32: the incoming block's weight, summed
    uint32_t pos{0};                // path steps of the lead's block played
    int8_t owed[k_max_actuators]{}; // steps made but not pulsed yet, signed
    uint8_t pin_dirs{0};
    uint8_t n{0};

    void reset()
    {
        lead.total= lead.own= 0;
        lead.blend_in= lead.blend_out= 0;
        for (uint8_t m = 0; m < k_max_actuators; m++) {
            lead.left[m]= 0;
            owed[m]= 0;
        }
        two= false;
        pos= 0;
    }

    void load(Player &p, const Block &b)
    {
        uint32_t longest= 0;
        for (uint8_t m = 0; m < n; m++) if(b.steps[m] > longest) longest= b.steps[m];
        for (uint8_t m = 0; m < n; m++) {
            p.left[m]= b.steps[m];
            p.share[m]= Block::share_of(b.steps[m], longest);
            p.acc[m]= 0x80000000UL;   // half a step in
        }
        p.own= 0;
        p.total= longest;
        p.blend_in= b.blend_in;
        p.blend_out= b.blend_out;
        p.dirs= b.direction_bits;
    }

    // motor m's step in p's next path step: -1, 0 or 1
    static int8_t advance(Player &p, uint8_t m)
    {
        if(p.left[m] == 0) return 0;
        uint32_t was= p.acc[m];
        p.acc[m]+= p.share[m];
        if(p.share[m] != 0 && p.acc[m] >= was) return 0;
        --p.left[m];
        return ((p.dirs >> m) & 1) ? -1 : 1;
    }

    void finish(Player &p, uint32_t upto)
    {
        while(p.own < upto) {
            ++p.own;
            for (uint8_t m = 0; m < n; m++) owed[m]+= advance(p, m);
        }
    }

    // a block with no blend into it
    void start(const Block &b)
    {
        load(lead, b);
        two= false;
        pos= 0;
    }

    bool wants_next() const
    {
        return !two && lead.blend_out != 0 && pos + lead.blend_out == lead.total;
    }

    // the window is as many path steps in the block coming in as in the one going out
    void open(const Block &next)
    {
        load(other, next);
        gain= 0x80000000UL / lead.blend_out;
        weight= gain >> 1;
        pick= 0x80000000UL;
        two= true;
        first_half= true;
        win_left= lead.blend_out;
    }

    void swap_at_mark()
    {
        Player t= lead;
        lead= other;
        other= t;
        first_half= false;
        win_left= lead.blend_in;
        pos= 0;
    }

    // One path step: the block it advances, for motor() to carry out
    Player *tick()
    {
        ++pos;
        Player *p= &lead;
        bool closing= false;
        if(two) {
            uint32_t was= pick;
            pick+= weight;
            weight+= gain;
            if((pick < was) == first_half) p= &other;
            closing= --win_left == 0 && !first_half;
        }
        if(p->own < p->total) {
            ++p->own;
        } else {
            p= nullptr;
        }
        if(closing) {
            // rounding leaves either a step short: the block going out ends here
            finish(other, other.total);
            finish(lead, pos);
            two= false;
        }
        return p;
    }

    // a pin pointing the other way is set now and pulsed a path step later: its setup time
    uint8_t motor(Player *p, uint8_t m)
    {
        int8_t d= p != nullptr ? advance(*p, m) : 0;
        int8_t o= owed[m];
        if(d == 0 && o == 0) return 0;
        bool pin= (pin_dirs >> m) & 1;
        if(o == 0 && (d < 0) == pin) return FIRE;
        o+= d;
        uint8_t does= 0;
        if(o != 0 && (o < 0) != pin) {
            pin_dirs^= 1 << m;
            does= TURNED;
        } else if(o != 0) {
            o+= o < 0 ? 1 : -1;
            does= FIRE;
        }
        owed[m]= o;
        return does;
    }

    bool busy(uint8_t m) const
    {
        return lead.left[m] != 0 || (two && other.left[m] != 0) || owed[m] != 0;
    }

    bool idle() const
    {
        for (uint8_t m = 0; m < n; m++) if(busy(m)) return false;
        return true;
    }

    bool owing() const
    {
        for (uint8_t m = 0; m < n; m++) if(owed[m] != 0) return true;
        return false;
    }

    void drop(uint8_t m)
    {
        lead.left[m]= 0;
        other.left[m]= 0;
        owed[m]= 0;
    }
};
