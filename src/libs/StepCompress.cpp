/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "StepCompress.h"
#include "StepStream.h"

float StepCompress::timer_hz_= 25000000.0F;
float StepCompress::tolerance_= 0.002F;

void StepCompress::configure(float timer_hz, float tolerance)
{
    timer_hz_= timer_hz;
    tolerance_= tolerance;
}

namespace {

struct Point {
    float v{0.0F};
    float a{0.0F};
};

// v(t) = v0 + a0 t + c2 t^2 + c3 t^3, reaching v1 at zero acceleration after distance D. Steps
// are marched from the last one, so the step after the previous call's costs one solve
struct Leg {
    bool valid{false};
    float v0, a0, v1, D, T, c2, c3;
    double t;          // summed over every step of the leg: a float loses the last step
    double s;
    uint32_t k;
    float ds;
    float dt;
    bool solved;

    float v_at(float tt) const { return v0 + (a0 + (c2 + c3 * tt) * tt) * tt; }
    float a_at(float tt) const { return a0 + (2.0F * c2 + 3.0F * c3 * tt) * tt; }
    bool at_end() const { return s >= D - 1e-6; }

    // T from D = T (v0 + v1) / 2 + a0 T^2 / 12
    void derive(const Point &p, const StepCompress::Target &tg)
    {
        v0= p.v;
        a0= p.a;
        v1= tg.v1;
        D= tg.d;
        float B= 0.5F * (v0 + v1);
        if(B < 1e-3F) B= 1e-3F;
        float disc= B * B + a0 * D / 3.0F;
        if(disc < 0.0F) {
            // a0 too negative for D leaves no real T: the start bends to the limit
            disc= 0.0F;
            a0= -3.0F * B * B / D;
        }
        T= 2.0F * D / (B + sqrtf(disc));
        float dv= v1 - v0;
        c2= (3.0F * dv - 2.0F * a0 * T) / (T * T);
        c3= (-2.0F * dv + a0 * T) / (T * T * T);
        t= 0.0;
        s= 0.0;
        k= 0;
        solved= false;
        valid= true;
    }

    void solve(float step)
    {
        ds= step;
        solved= true;
        if(s + step >= D - 0.5 * step) {
            // the last step lands on T exactly; a leg to rest has v1 = 0 to divide by
            float rest= T - (float)t;
            dt= rest > 0.0F ? rest : step / (v1 > 1e-3F ? v1 : 1e-3F);
            return;
        }

        float tt= (float)t;
        float v= v_at(tt);
        float a= a_at(tt);
        float j= 2.0F * c2 + 6.0F * c3 * tt;
        float s4= 6.0F * c3;
        if(v < 0.0F) v= 0.0F;

        // step/v is no seed where the speed is too small to carry the step: use the constant-jerk time
        float d= v > 1e-3F ? step / v : 1.0F;
        if(j > 0.0F && v * v * v < j * step * step) {
            float dj= cbrtf(6.0F * step / j);
            if(dj < d) d= dj;
        }
        for (uint8_t i = 0; i < 8; i++) {
            float g= (((s4 / 24.0F * d + j / 6.0F) * d + a * 0.5F) * d + v) * d - step;
            float dg= ((s4 / 6.0F * d + j * 0.5F) * d + a) * d + v;
            if(dg <= 0.0F) break;
            float move= g / dg;
            d-= move;
            if(fabsf(move) < 1e-7F * d) break;
        }
        dt= d > 0.0F ? d : step / (v > 1e-3F ? v : 1e-3F);
    }

    void advance_to(uint32_t to, float step)
    {
        while(k < to) {
            if(!solved) solve(step);
            t+= dt;
            s+= ds;
            k++;
            solved= false;
        }
    }

    // forward only: at is never behind the step the leg stands at
    float interval(uint32_t at, float step)
    {
        advance_to(at, step);
        if(!solved) solve(step);
        return dt;
    }

    Point point() const
    {
        Point p;
        if(at_end()) {
            p.v= v1;
            p.a= 0.0F;
        } else {
            p.v= v_at((float)t);
            p.a= a_at((float)t);
        }
        return p;
    }
};

Point point;
Leg leg;

}

void StepCompress::rewind()
{
    point= Point();
    leg.valid= false;
}

float StepCompress::profile_v() { return leg.valid ? leg.point().v : point.v; }
float StepCompress::profile_a() { return leg.valid ? leg.point().a : point.a; }

uint32_t StepCompress::plateau(StepStream &out, float v, float ds, uint32_t steps)
{
    if(v < 1e-3F) {
        return 0;
    }
    point.v= v;
    point.a= 0.0F;
    leg.valid= false;

    uint32_t interval= (uint32_t)(timer_hz_ * ds / v + 0.5F);
    if(interval < 1) interval= 1;
    uint32_t done= 0;
    while(done < steps) {
        if(out.full()) {
            break;
        }
        uint32_t n= steps - done;
        out.push(interval, n, 0);
        done+= n;
    }
    return done;
}

uint32_t StepCompress::ramp(StepStream &out, const Target &t, float ds, uint32_t steps, uint32_t done)
{
    // a new leg starts from where the profile stands, so speed and acceleration stay continuous
    bool same= leg.valid && fabsf(leg.v1 - t.v1) <= 1e-3F * (t.v1 + 1.0F)
               && fabs((leg.D - leg.s) - t.d) <= 0.5 * ds + 1e-4;
    if(!same) {
        if(leg.valid) point= leg.point();
        leg.derive(point, t);
    }
    uint32_t base= leg.k - done;
    const float hz= timer_hz_;

    uint32_t i= done;
    while(i < steps) {
        if(out.full()) {
            break;
        }

        float first= leg.interval(base + i, ds) * hz;
        if(i + 1 >= steps) {
            out.push((uint32_t)(first + 0.5F), 1, 0);
            i++;
            break;
        }

        float last= leg.interval(base + i + 1, ds) * hz;
        float add= last - first;
        uint32_t k= 2;
        float want= first + last;

        while(i + k < steps && k < 0xFFFFU) {
            float exact= leg.interval(base + i + k, ds) * hz;
            float allow= exact * tolerance_;
            if(allow < 1.0F) allow= 1.0F;
            if(fabsf(first + k * add - exact) > allow) {
                break;
            }
            want+= exact;
            last= exact;
            k++;
        }

        float fit= (want - k * first) / (k * (k - 1) * 0.5F);
        float allow= last * tolerance_;
        if(allow < 1.0F) allow= 1.0F;
        if(fabsf(first + (k - 1) * fit - last) > allow) {
            fit= add;
        }

        float scaled= fit * (1 << StepStream::k_add_shift);
        out.push((uint32_t)(first + 0.5F), k, (int32_t)(scaled < 0 ? scaled - 0.5F : scaled + 0.5F));
        i+= k;
    }
    // the next span, or a new target, starts from where the ring stands
    leg.advance_to(base + i, ds);
    return i;
}
