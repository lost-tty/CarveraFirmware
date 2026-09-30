// The speed profile along the path, compressed into step intervals and played back the way
// the interrupt plays them.
//
// Every leg is a quintic in time: acceleration and jerk zero at both ends, 15/8 of the
// planner's mean in the middle, same distance and time as the constant-acceleration ramp. A
// leg runs to the next point where the plan changes phase, across as many blocks as keep it.
//
// What has to hold, or the axis loses steps, arrives at the wrong time, or jolts:
//   - the step count is exactly what the spans ask for
//   - a single leg takes the planner's 2 d / (v0 + v1), starts at v0 and ends at v1
//   - its acceleration never exceeds 15/8 of the mean and is near zero at both ends
//   - a ramp down is the ramp up in reverse
//   - a ramp from rest starts, with a first step at the constant-snap time
//   - a chain of short blocks that accelerate together is one leg: one peak, no dip between
//     the blocks, whatever their step lengths
//   - a leg whose end moves while it is being written stays continuous in speed and acceleration
//   - a junction without slack ends the leg there
//
// c++ -std=c++11 -I ../src/libs stepcompress_test.cpp ../src/libs/StepCompress.cpp
#include "StepCompress.h"
#include "StepStream.h"

#include <cstdio>
#include <cmath>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

static const double HZ = 25000000.0;
typedef StepCompress::Span Span;
typedef StepCompress::Target Target;

// the plan as feed_stream walks it: blocks with their spans, written in order, the ring
// drained whenever it fills
struct Plan {
    std::vector<Span> spans;
    std::vector<double> dt;         // what was played, in seconds
    std::vector<size_t> block_end;  // index in dt where each block ends

    bool next_from(size_t i, uint8_t j, Span &out) const
    {
        if (i + j >= spans.size()) return false;
        out = spans[i + j];
        return true;
    }

    void drain(StepStream &ring)
    {
        for (;;) {
            uint32_t t = ring.take();
            if (t == 0) break;
            dt.push_back(t / HZ);
        }
    }

    void run(size_t from = 0)
    {
        StepStream ring;
        for (size_t i = from; i < spans.size(); i++) {
            const Span s = spans[i];
            uint32_t total = s.up + s.flat + s.down, at = 0;
            auto next = [&](uint8_t j, Span &o) { return next_from(i, j, o); };
            while (at < total) {
                if (at < s.up) {
                    Target t = StepCompress::target(StepCompress::ACCEL, at, s, next);
                    at = StepCompress::ramp(ring, t, s.ds, s.up, at);
                } else if (at < s.up + s.flat) {
                    at += StepCompress::plateau(ring, s.v_flat, s.ds, s.up + s.flat - at);
                } else {
                    Target t = StepCompress::target(StepCompress::DECEL, at, s, next);
                    at = s.up + s.flat + StepCompress::ramp(ring, t, s.ds, s.down, at - s.up - s.flat);
                }
                drain(ring);
            }
            block_end.push_back(dt.size());
        }
    }
};

// one block that is entirely one ramp
static Span ramp_span(float ds, uint32_t steps, float v0, float v1, float accel)
{
    Span s;
    s.ds = ds;
    s.up = v1 > v0 ? steps : 0;
    s.down = v1 > v0 ? 0 : steps;
    s.flat = 0;
    s.v_entry = v0;
    s.v_flat = v1 > v0 ? v1 : v0;
    s.v_exit = v1;
    s.v_max_entry = 1e9F;
    s.accel = accel;
    return s;
}

// acceleration over windows of w steps, from the played intervals in [a, b), in steps/s^2
static std::vector<double> windowed(const std::vector<double> &dt, size_t a, size_t b, size_t w)
{
    std::vector<double> out;
    for (size_t i = a; i + w < b; i++) {
        double t_w = 0;
        for (size_t j = 0; j < w; j++) t_w += dt[i + j];
        out.push_back((1.0 / dt[i + w] - 1.0 / dt[i]) / t_w);
    }
    return out;
}

static void check_single(float v0, float v1, uint32_t steps)
{
    const float ds = 0.005F;   // 200 steps/mm
    float mean = fabsf(v1 * v1 - v0 * v0) / (2.0F * steps * ds);
    Plan p;
    p.spans.push_back(ramp_span(ds, steps, v0, v1, mean));
    StepCompress::rewind();
    if (v0 > 0) {
        StepStream ring;
        StepCompress::plateau(ring, v0, ds, 1);   // the profile stands at v0 before the leg
    }
    p.run();
    const std::vector<double> &dt = p.dt;
    CHECK(dt.size() == steps);
    if (dt.size() != steps) return;

    double T = 2.0 * steps * ds / (v0 + v1);
    double total = 0;
    for (double d : dt) total += d;
    CHECK(fabs(total - T) < 0.002 * T);

    double first = ds / dt.front(), last = ds / dt.back();
    if (v0 > 0) CHECK(fabs(first - v0) < 0.05 * v0 + 0.01);
    if (v1 > 0) CHECK(fabs(last - v1) < 0.05 * v1 + 0.01);

    size_t w = steps / 50;
    if (w < 2) w = 2;
    std::vector<double> a = windowed(dt, 0, dt.size(), w);
    for (double &x : a) x = fabs(x) * ds;   // to mm/s^2
    double peak = 0;
    for (double x : a) if (x > peak) peak = x;
    float top = v0 > v1 ? v0 : v1;
    printf("  %6.1f -> %6.1f mm/s over %5u: peak %.2fx mean, ends %.2fx %.2fx\n", v0, v1, steps,
           peak / mean, a.front() / mean, a.back() / mean);
    if (steps >= 50) {
        CHECK(peak < 1.875 * mean * 1.05);
        CHECK(peak > 1.875 * mean * 0.9);
        // an end at rest is left out: its last 2% of the distance is a third of the leg's
        // time under constant snap, and the first-step check covers it
        if (v0 > 0.1F * top) CHECK(a.front() < 0.25 * mean);
        if (v1 > 0.1F * top) CHECK(a.back() < 0.25 * mean);
    }
}

int main()
{
    const float V = 30.0F;   // mm/s
    const float ds = 0.005F;

    // 1. single legs: up, down, from rest, to rest, short, long, tiny
    check_single(5.0F, V, 2000);
    check_single(V, 5.0F, 2000);
    check_single(0.0F, V, 3000);
    check_single(V, 0.0F, 3000);
    check_single(10.0F, 12.5F, 50);
    check_single(2.5F, 200.0F, 20000);
    check_single(1.5F, 1.6F, 3);
    printf("single legs: ok\n");

    // 2. the way down is the way up, reversed
    {
        Plan up, down;
        up.spans.push_back(ramp_span(ds, 2000, 5.0F, V, 1.0F));
        down.spans.push_back(ramp_span(ds, 2000, V, 5.0F, 1.0F));
        StepStream ring;
        StepCompress::rewind(); StepCompress::plateau(ring, 5.0F, ds, 1); up.run();
        StepCompress::rewind(); StepCompress::plateau(ring, V, ds, 1);    down.run();
        double worst = 0;
        for (size_t i = 0; i < up.dt.size(); i++) {
            double e = fabs(up.dt[i] - down.dt[down.dt.size() - 1 - i]) / up.dt[i];
            if (e > worst) worst = e;
        }
        CHECK(worst < 0.01);
        printf("symmetry: ok (worst %.4f)\n", worst);
    }

    // 3. from rest: the first step comes at the constant-snap time. s = snap d^4 / 24 with
    //    snap = 60 dv / T^3, so d = (0.4 T^3 ds / dv)^(1/4)
    {
        Plan p;
        p.spans.push_back(ramp_span(ds, 3000, 0.0F, V, 1.0F));
        StepCompress::rewind();
        p.run();
        double T = 2.0 * 3000 * ds / V;
        double first = pow(0.4 * T * T * T * ds / V, 0.25);
        CHECK(fabs(p.dt[0] - first) < 0.1 * first);
        printf("from rest: first step at %.1f ms, expected %.1f ms\n", p.dt[0] * 1e3, first * 1e3);
    }

    // 4. a chain of 30 chords of 1 mm accelerating from rest to 30 mm/s is one leg: one peak,
    //    and no dip between chords. Chords alternate between two step lengths, as the longest
    //    axis changes around an arc. Planned as the planner would: v^2 = 2 a s
    {
        const float a_mean = 15.0F;   // 30 mm to reach 30 mm/s
        Plan p;
        float v = 0;
        for (int i = 0; i < 30; i++) {
            float dsi = (i & 1) ? 0.005F : 0.007F;
            uint32_t n = (uint32_t)(1.0F / dsi + 0.5F);
            float vn = sqrtf(v * v + 2.0F * a_mean * n * dsi);
            p.spans.push_back(ramp_span(dsi, n, v, vn, a_mean));
            v = vn;
        }
        // then a plateau block, so the chain has a real end
        Span flat = ramp_span(ds, 1000, v, v, a_mean);
        flat.up = 0; flat.down = 0; flat.flat = 1000; flat.v_flat = v;
        p.spans.push_back(flat);
        StepCompress::rewind();
        p.run();
        size_t chain = p.block_end[29];
        CHECK(p.dt.size() == chain + 1000);

        // the speed at each played step, in mm/s, with the block's own step length
        std::vector<double> vs;
        size_t b = 0;
        for (size_t i = 0; i < chain; i++) {
            while (i >= p.block_end[b]) b++;
            vs.push_back(p.spans[b].ds / p.dt[i]);
        }
        // acceleration over 40-step windows: one rise and one fall, nothing in between. A
        // leg per chord would dip to zero 29 times
        std::vector<double> a;
        for (size_t i = 0; i + 40 < chain; i++) {
            double t_w = 0;
            for (size_t j = 0; j < 40; j++) t_w += p.dt[i + j];
            a.push_back((vs[i + 40] - vs[i]) / t_w);
        }
        double peak = 0;
        size_t at = 0;
        for (size_t i = 0; i < a.size(); i++) if (a[i] > peak) { peak = a[i]; at = i; }
        double tol = 0.15 * a_mean;   // whole-tick intervals at 6000 steps/s
        int dips = 0;
        for (size_t i = 0; i + 1 < a.size(); i++) {
            if (i < at && a[i + 1] < a[i] - tol) dips++;
            if (i >= at && a[i + 1] > a[i] + tol) dips++;
        }
        double v_end = ds / p.dt[chain];
        printf("chain: peak %.2fx mean at %zu%% of the distance, %d dips, end %.2f of %.2f mm/s\n",
               peak / a_mean, at * 100 / a.size(), dips, v_end, v);
        CHECK(fabs(v_end - v) < 0.02 * v);
        CHECK(dips == 0);
        CHECK(peak < 1.875 * a_mean * 1.15);
        CHECK(peak > 1.875 * a_mean * 0.85);
    }

    // 5. a junction without slack ends the leg: the same chain with one tight corner
    {
        Plan p;
        float v = 0;
        for (int i = 0; i < 30; i++) {
            float vn = sqrtf(v * v + 2.0F * 15.0F * 1.0F);
            Span s = ramp_span(ds, 200, v, vn, 15.0F);
            if (i == 15) s.v_max_entry = v * 1.05F;   // the corner: the plan sits on its limit
            p.spans.push_back(s);
            v = vn;
        }
        Span s = p.spans[0];
        Target t = StepCompress::target(StepCompress::ACCEL, 0, s,
                                        [&](uint8_t j, Span &o) { return p.next_from(0, j, o); });
        CHECK(fabs(t.d - 15.0F) < 1e-3F);
        CHECK(fabs(t.v1 - p.spans[14].v_exit) < 1e-3F);
        printf("tight corner: leg ends at %.1f mm, %.2f mm/s\n", t.d, t.v1);
    }

    // 6. the end moves while the leg is being written: the plan grows. Speed and acceleration
    //    stay continuous at the cut
    {
        Plan p;
        p.spans.push_back(ramp_span(ds, 4000, 5.0F, 20.0F, 1.0F));   // 20 mm to 20 mm/s
        StepStream ring;
        StepCompress::rewind();
        StepCompress::plateau(ring, 5.0F, ds, 1);
        p.drain(ring);
        p.dt.clear();
        // write the first 10 mm against that plan
        Span s = p.spans[0];
        Target t = StepCompress::target(StepCompress::ACCEL, 0, s,
                                        [&](uint8_t j, Span &o) { return p.next_from(0, j, o); });
        uint32_t at = 0;
        while (at < 2000) {
            at = StepCompress::ramp(ring, t, ds, 2000, at);
            p.drain(ring);
        }
        float v_cut = StepCompress::profile_v(), a_cut = StepCompress::profile_a();
        // now the plan says: keep accelerating to 30 mm/s over 60 mm from here
        Target t2;
        t2.v1 = 30.0F;
        t2.d = 60.0F;
        uint32_t n2 = 12000;
        at = 0;
        while (at < n2) {
            at = StepCompress::ramp(ring, t2, ds, n2, at);
            p.drain(ring);
        }
        CHECK(p.dt.size() == 2000 + 12000);
        std::vector<double> a = windowed(p.dt, 1900, 2100, 20);
        double before = a.front() * ds, after = a.back() * ds;
        double v_before = ds / p.dt[1999], v_after = ds / p.dt[2000];
        printf("cut at %.2f mm/s, %.2f mm/s^2: speed %.2f -> %.2f, accel %.2f -> %.2f\n",
               v_cut, a_cut, v_before, v_after, before, after);
        CHECK(fabs(v_after - v_before) < 0.01 * v_before);
        CHECK(fabs(after - before) < 0.15 * fabs(before) + 0.2);
        CHECK(fabs(ds / p.dt.back() - 30.0) < 0.6);
    }

    if (fails == 0) printf("all ok\n");
    return fails == 0 ? 0 : 1;
}
