/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include <math.h>

class StepStream;

// Every ramp is a quintic in time with zero acceleration and zero jerk at its ends, run across
// the blocks that continue it: a constant-acceleration ramp jolts the frame at every phase
// change and block boundary, and a leg with finite jerk at its ends rings it at every stop.
// Its peak is 15/8 of the mean the planner plans distances with.
class StepCompress
{
public:
    static constexpr float k_peak_over_mean= 1.875F;

    static void configure(float timer_hz, float tolerance);
    static void rewind();

    // mm per path step, the three spans in steps, speeds in mm/s
    struct Span {
        float ds;
        uint32_t up, flat, down;
        float v_entry, v_flat, v_exit;
        float v_max_entry, accel;

        // the span starts at step at0 of the block's `whole`
        uint32_t at0{0}, whole{0};

        float dist(uint32_t a, uint32_t b) const { return (float)(b - a) * ds; }
    };
    enum Kind { ACCEL, DECEL };
    struct Target { float v1; float d; };

    // next(j, s) fills the j-th block after `first`, false where the plan ends
    template<class F>
    static Target target(Kind kind, uint32_t at, const Span &first, F next)
    {
        Target t;
        Span prev= first;
        if(kind == ACCEL) {
            t.d= first.dist(at, first.up);
            if(first.flat + first.down != 0) { t.v1= first.v_flat; return t; }
        } else {
            t.d= first.dist(at, first.up + first.flat + first.down);
        }
        t.v1= first.v_exit;
        for (uint8_t j = 1; ; j++) {
            Span s;
            if(!next(j, s) || !mergeable(prev, s)) return t;
            if(kind == ACCEL) {
                if(s.up == 0) return t;
                t.d+= s.dist(0, s.up);
                if(s.flat + s.down != 0) { t.v1= s.v_flat; return t; }
            } else {
                if(s.up + s.flat != 0) return t;
                t.d+= s.dist(0, s.down);
            }
            t.v1= s.v_exit;
            prev= s;
        }
    }

    // `steps` of the span, `done` of them already written
    static uint32_t ramp(StepStream &out, const Target &t, const Span &s,
                         uint32_t steps, uint32_t done);
    static uint32_t plateau(StepStream &out, float v, const Span &s, uint32_t steps);

    static float profile_v();
    static float profile_a();

private:
    // The quintic runs up to 1.3x faster than the planner's constant-acceleration ramp through
    // the second half of a leg, so a junction inside a leg needs that slack below its limit;
    // and a leg has one acceleration, so the blocks in it have to agree on theirs
    static bool mergeable(const Span &a, const Span &b)
    {
        return b.v_entry * 1.35F <= b.v_max_entry && fabsf(b.accel - a.accel) <= 0.1F * a.accel;
    }

    static float timer_hz_;
    static float tolerance_;
};
