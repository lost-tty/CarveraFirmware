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
    float j{0.0F};
};

// v(t) = v0 + a0 t + j0 t^2/2 + c3 t^3 + c4 t^4 + c5 t^5: v1 at D, acceleration and jerk 0
struct Leg {
    bool valid{false};
    float v0, a0, j0, v1, D, T, c3, c4, c5;
    double t;   // a float loses the last step
    double s;
    float v, a, j, s4;
    float tf, left;
    float taylor[6];   // the distance from here in time, highest power first
    bool located;
    float hint;

    float v_at(float tt) const
    {
        return v0 + (a0 + (0.5F * j0 + (c3 + (c4 + c5 * tt) * tt) * tt) * tt) * tt;
    }
    float a_at(float tt) const
    {
        return a0 + (j0 + (3.0F * c3 + (4.0F * c4 + 5.0F * c5 * tt) * tt) * tt) * tt;
    }
    float j_at(float tt) const
    {
        return j0 + (6.0F * c3 + (12.0F * c4 + 20.0F * c5 * tt) * tt) * tt;
    }
    bool at_end() const { return s >= D - 1e-6; }

    // T from D = T (v0 + v1) / 2 + a0 T^2 / 10 + j0 T^3 / 120
    void derive(const Point &p, const StepCompress::Target &tg)
    {
        v0= p.v;
        a0= p.a;
        j0= p.j;
        v1= tg.v1;
        D= tg.d;
        float B= 0.5F * (v0 + v1);
        if(B < 1e-3F) B= 1e-3F;
        T= D / B;
        bool ok= false;
        for (uint8_t i = 0; i < 8 && T > 0.0F; i++) {
            float f= (B + (a0 / 10.0F + j0 / 120.0F * T) * T) * T - D;
            float df= B + (a0 / 5.0F + j0 / 40.0F * T) * T;
            if(df <= 0.0F) break;
            T-= f / df;
            ok= fabsf(f) <= 1e-6F * D;
            if(ok) break;
        }
        if(!ok || T <= 0.0F) {
            // no real T with this jerk: drop it, and if the acceleration alone leaves none
            // either, bend the start to the limit
            j0= 0.0F;
            float disc= B * B + 0.4F * a0 * D;
            if(disc < 0.0F) {
                disc= 0.0F;
                a0= -2.5F * B * B / D;
            }
            T= 2.0F * D / (B + sqrtf(disc));
        }
        float dv= v1 - v0;
        float T2= T * T, T3= T2 * T;
        c3= (10.0F * dv - 6.0F * a0 * T - 1.5F * j0 * T2) / T3;
        c4= (-15.0F * dv + 8.0F * a0 * T + 1.5F * j0 * T2) / (T3 * T);
        c5= (6.0F * dv - 3.0F * a0 * T - 0.5F * j0 * T2) / (T3 * T2);
        t= 0.0;
        s= 0.0;
        hint= 0.0F;
        located= false;
        valid= true;
    }

    void locate()
    {
        if(located)
            return;

        float tt= (float)t;
        tf= tt;
        left= (float)(D - s);
        v= v_at(tt);
        a= a_at(tt);
        j= j_at(tt);
        s4= 6.0F * c3 + (24.0F * c4 + 60.0F * c5 * tt) * tt;
        if(v < 0.0F)
            v= 0.0F;

        taylor[0]= c5 * (1.0F / 6.0F);
        taylor[1]= 0.2F * c4 + c5 * tt;
        taylor[2]= s4 * (1.0F / 24.0F);
        taylor[3]= j * (1.0F / 6.0F);
        taylor[4]= 0.5F * a;
        taylor[5]= v;
        located= true;
    }

    __attribute__((noinline)) float time_to(float d, float step, float seed, float &speed) const
    {
        if(v1 < 1e-3F && d + 0.5F * step >= left) {
            // to rest: the last step is what is left of T, no solve at v = 0
            float rest= T - tf;
            speed= 0.0F;
            return rest > 0.0F ? rest : d / 1e-3F;
        }

        float u= seed;
        if(u <= 0.0F) {
            float r= v * v + 2.0F * a * d;
            u= r > 0.0F ? 2.0F * d / (v + sqrtf(r)) : v > 1e-3F ? d / v : 1.0F;
            if(j > 0.0F && v * u < 0.5F * d) {
                float uj= cbrtf(6.0F * d / j);
                if(uj < u)
                    u= uj;
            }
            if(s4 > 0.0F && v * u < 0.5F * d) {
                float us= sqrtf(sqrtf(24.0F * d / s4));
                if(us < u)
                    u= us;
            }
        }
        speed= v;
        for (uint8_t i = 0; i < 8; i++) {
            // p the distance, q its derivative
            float p= taylor[0], q= taylor[0];
            for (uint8_t k = 1; k < 6; k++) {
                p= p * u + taylor[k];
                q= q * u + p;
            }
            float g= p * u - d, dg= q;
            if(dg <= 0.0F)
                break;

            speed= dg;
            float move= g / dg;
            u-= move;
            if(fabsf(move) <= 4e-7F * u)
                break;
        }
        return u > 0.0F ? u : d / (v > 1e-3F ? v : 1e-3F);
    }

    float interval_at(float d, float step, float before, float to, float speed) const
    {
        float unused;
        float seed= speed > 1e-3F ? to - step / speed : 0.0F;
        return to - time_to(d - step, before, seed, unused);
    }

    uint32_t reach(float step, float tol, uint32_t most) const
    {
        if(most <= 2 || v < 1e-3F)
            return most < 2 ? most : 2;

        float bend= fabsf(3.0F * a * a - j * v);
        float n= v * v * sqrtf(4.0F * tol / (step * step * (bend + 1e-9F)));
        if(n > (float)most)
            n= (float)most;

        float tt= tf + n * step / v;
        if(tt > T)
            tt= T;

        float ve= v_at(tt), ae= a_at(tt), je= j_at(tt);
        if(ve < 1e-3F)
            return 2;

        float slow= ve < v ? ve : v;
        float bend_e= fabsf(3.0F * ae * ae - je * ve);
        if(bend_e > bend)
            bend= bend_e;

        n= slow * slow * sqrtf(4.0F * tol / (step * step * (bend + 1e-9F)));
        if(n > (float)most)
            return most;

        return n < 2.0F ? 2 : (uint32_t)n;
    }

    void advance(float d, float seconds, float last)
    {
        t+= seconds;
        s+= d;
        hint= last;
        located= false;
    }

    Point point() const
    {
        Point p;
        if(at_end()) {
            p.v= v1;
            p.a= 0.0F;
            p.j= 0.0F;
        } else {
            p.v= v_at((float)t);
            p.a= a_at((float)t);
            p.j= j_at((float)t);
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

static bool within(float played, float exact, float tolerance)
{
    float allow= exact * tolerance;
    if(allow < 1.0F)
        allow= 1.0F;

    return fabsf(played - exact) <= allow;
}

uint32_t StepCompress::plateau(StepStream &out, float v, const Span &s, uint32_t steps)
{
    if(v < 1e-3F) {
        return 0;
    }
    point.v= v;
    point.a= 0.0F;
    point.j= 0.0F;
    leg.valid= false;

    if(steps == 0 || out.full()) {
        return 0;
    }
    uint32_t interval= (uint32_t)(timer_hz_ / v * s.ds + 0.5F);
    if(interval < 1)
        interval= 1;
    out.push(interval, steps, 0);
    return steps;
}

uint32_t StepCompress::ramp(StepStream &out, const Target &t, const Span &s, uint32_t steps,
                            uint32_t done)
{
    // a new leg starts from where the profile stands, so speed and acceleration stay continuous
    bool same= leg.valid && fabsf(leg.v1 - t.v1) <= 1e-3F * (t.v1 + 1.0F)
               && fabs((leg.D - leg.s) - t.d) <= 0.5 * s.ds + 1e-4;
    if(!same) {
        if(leg.valid) point= leg.point();
        leg.derive(point, t);
    }
    const float hz= timer_hz_;

    uint32_t i= done;
    while(i < steps) {
        if(out.full()) {
            break;
        }
        leg.locate();

        float step= s.ds;
        float speed;
        float first= leg.time_to(step, step, leg.hint, speed);
        uint32_t ticks= (uint32_t)(first * hz + 0.5F);

        uint32_t most= steps - i;
        if(most > 0xFFFFU)
            most= 0xFFFFU;
        if(most < 2) {
            out.push(ticks, 1, 0);
            leg.advance(step, first, first);
            i++;
            continue;
        }

        const float half= 0.5F * tolerance_;
        uint32_t n= leg.reach(step, tolerance_, most);
        float d, total, last;
        int32_t add;
        for (;;) {
            d= (float)n * step;
            total= leg.time_to(d, step, 0.0F, speed);
            last= total - first;
            float mean= total / (float)n * hz;
            if(n > 2) {
                last= leg.interval_at(d, step, step, total, speed);
                ticks= (uint32_t)(mean - 0.5F * (last - first) * hz + 0.5F);
            }
            float fit= 2.0F * (mean - (float)ticks) / (float)(n - 1);
            float scaled= fit * (1 << StepStream::k_add_shift);
            add= (int32_t)(scaled < 0 ? scaled - 0.5F : scaled + 0.5F);
            if(n == 2)
                break;

            // the line misses the first and the last interval by the same amount; a cubic
            // error peaks at 0.21 of the run
            float slope= (float)add / (1 << StepStream::k_add_shift);
            float e= fabsf(mean - 0.5F * (first + last) * hz);
            e+= 0.5F * (float)(n - 1) / (1 << StepStream::k_add_shift);
            bool ok= e <= fmaxf(1.0F, half * hz * fminf(first, last));
            if(ok && n > 8) {
                uint32_t m= (uint32_t)(0.211F * (float)n) + 1;
                float dm= (float)m * step;
                float to= leg.time_to(dm, step, 0.0F, speed);
                float mid= leg.interval_at(dm, step, step, to, speed);
                ok= within((float)ticks + (float)(m - 1) * slope, mid * hz, half);
            }
            if(ok)
                break;

            n= n * 5 / 8;
            if(n < 2)
                n= 2;
        }

        out.push(ticks, n, add);
        leg.advance(d, total, last);
        i+= n;
    }
    return i;
}
