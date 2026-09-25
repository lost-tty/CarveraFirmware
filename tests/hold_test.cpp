// The feed hold caps the path rate down to a stand: does the cap reach zero, does the distance
// match v^2/2a, does a block boundary change anything, and do the motors derived from the path
// stay on the line and land on their last step?
// c++ -std=c++11 hold_test.cpp
#include <cstdio>
#include <cstdint>
#include <cmath>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

#define STEPTICKER_FPSCALE (1LL<<62)
static const double TICK_HZ = 100000.0;
static const double STEPS_PER_MM = 200.0;
static const int MOTORS = 3;

static int64_t rate_to_fp(double steps_per_sec)
{
    return (int64_t)round((steps_per_sec / TICK_HZ) * STEPTICKER_FPSCALE);
}

static int64_t accel_to_fp(double steps_per_s2)
{
    double fp_scale = (double)STEPTICKER_FPSCALE / (TICK_HZ * TICK_HZ);
    return (int64_t)round(steps_per_s2 * fp_scale);
}

// Block::set_ratios
static uint32_t ratio_of(uint32_t steps, uint32_t longest)
{
    return steps == longest ? 0 : (uint32_t)((((uint64_t)steps << 32) + longest / 2) / longest);
}

// StepTicker::owed
static uint32_t owed(uint32_t path_steps, uint32_t path_frac, uint32_t ratio)
{
    uint64_t v = (uint64_t)path_steps * ratio + (((uint64_t)path_frac * ratio) >> 32) + (1u << 31);
    return (uint32_t)(v >> 32);
}

// what a block hands the ticker
struct Ramp {
    int64_t steps_per_tick;      // rate at block start
    int64_t deceleration_change; // negative
};

// StepTicker::path and StepTicker::state
struct Ticker {
    int64_t steps_per_tick, counter;
    uint32_t steps, step_count;
    struct { uint32_t steps_to_move, step_count, ratio; } m[MOTORS];

    void start(const Ramp &ramp, const uint32_t steps_of[MOTORS])
    {
        steps = 0;
        for(int i = 0; i < MOTORS; i++) if(steps_of[i] > steps) steps = steps_of[i];
        for(int i = 0; i < MOTORS; i++) m[i] = {steps_of[i], 0, ratio_of(steps_of[i], steps)};
        step_count = 0;
        counter = 0;
        steps_per_tick = ramp.steps_per_tick;
    }

    // the tick's path and motor part; returns true while the block has steps left
    bool tick(int64_t rate)
    {
        counter += rate;
        if(counter >= STEPTICKER_FPSCALE) {
            counter -= STEPTICKER_FPSCALE;
            step_count++;
        }
        uint32_t frac = (uint32_t)(counter >> 30);
        bool moving = false;
        for(int i = 0; i < MOTORS; i++) {
            if(m[i].steps_to_move == 0) continue;
            uint32_t due = m[i].ratio == 0 ? step_count : owed(step_count, frac, m[i].ratio);
            if(due > m[i].steps_to_move) due = m[i].steps_to_move;
            if(due > m[i].step_count) {
                m[i].step_count++;
                if(m[i].step_count == m[i].steps_to_move) { m[i].steps_to_move = 0; continue; }
            }
            moving = true;
        }
        return moving;
    }
};

struct Run { double mm; uint32_t ticks; bool stopped; };

// StepTicker::step_tick, hold path only: the block ramp is left flat so the test measures the
// hold and nothing else
static Run hold_from(double v_mm_s, double decel_mm_s2, uint32_t block_steps, uint32_t blocks)
{
    Ramp ramp;
    ramp.steps_per_tick = rate_to_fp(v_mm_s * STEPS_PER_MM);
    ramp.deceleration_change = -accel_to_fp(decel_mm_s2 * STEPS_PER_MM);

    const uint32_t steps_of[MOTORS] = {block_steps, 0, 0};
    Ticker t;
    t.start(ramp, steps_of);

    // StepTicker::brake: the cap starts at the rate the path has now
    int64_t hold_rate = t.steps_per_tick;
    bool holding = true;

    uint32_t ticks = 0, total_steps = 0, block = 0;

    while(ticks < 10u * (uint32_t)TICK_HZ) {   // a tenth of a second is plenty at these rates
        ticks++;

        // the tick's hold branch
        if(holding) {
            int64_t a = -ramp.deceleration_change;
            hold_rate = hold_rate > a ? hold_rate - a : 0;
            if(hold_rate == 0) return Run{total_steps / STEPS_PER_MM, ticks, true};
        }

        int64_t use = holding && hold_rate < t.steps_per_tick ? hold_rate : t.steps_per_tick;
        uint32_t before = t.m[0].step_count;
        bool moving = t.tick(use);
        total_steps += t.m[0].step_count - before;

        if(!moving) {   // block ran out: the next one starts
            if(++block >= blocks) return Run{total_steps / STEPS_PER_MM, ticks, false};
            t.start(ramp, steps_of);
        }
    }
    return Run{total_steps / STEPS_PER_MM, ticks, false};
}

int main()
{
    // 1. it stops, and within the time v/a says
    {
        Run r = hold_from(50.0, 1000.0, 1000000, 1);
        CHECK(r.stopped);
        double expect_s = 50.0 / 1000.0;
        double got_s = r.ticks / TICK_HZ;
        CHECK(fabs(got_s - expect_s) < expect_s * 0.05);
        printf("50 mm/s at 1000 mm/s2: stopped after %.1f ms (expect %.1f), %.3f mm\n",
               got_s * 1000, expect_s * 1000, r.mm);
    }

    // 2. the distance is v^2/2a
    {
        Run r = hold_from(50.0, 1000.0, 1000000, 1);
        double expect_mm = (50.0 * 50.0) / (2 * 1000.0);
        CHECK(fabs(r.mm - expect_mm) < expect_mm * 0.1);
        printf("braking distance %.3f mm (expect %.3f)\n", r.mm, expect_mm);
    }

    // 3. a block that runs out part way through does not change the outcome
    {
        Run one = hold_from(50.0, 1000.0, 1000000, 1);
        Run many = hold_from(50.0, 1000.0, 20, 500);   // a boundary every 0.1 mm
        CHECK(many.stopped);
        CHECK(fabs(many.mm - one.mm) < one.mm * 0.1);
        printf("across %u block boundaries: %.3f mm (one block: %.3f)\n",
               (unsigned)(many.mm * STEPS_PER_MM / 20), many.mm, one.mm);
    }

    // 4. a slow move stops too, and sooner
    {
        Run slow = hold_from(5.0, 1000.0, 1000000, 1);
        Run fast = hold_from(50.0, 1000.0, 1000000, 1);
        CHECK(slow.stopped);
        CHECK(slow.ticks < fast.ticks);
        printf("5 mm/s stops in %.1f ms, 50 mm/s in %.1f ms\n",
               slow.ticks / TICK_HZ * 1000, fast.ticks / TICK_HZ * 1000);
    }

    // 5. the hold actually brakes: it does not end on the first tick with nothing moved
    {
        Run r = hold_from(50.0, 1000.0, 1000000, 1);
        CHECK(r.ticks > 100);   // a cap that starts at zero would end here on tick 1
        CHECK(r.mm > 0.1);
        printf("braked over %u ticks, %.3f mm\n", r.ticks, r.mm);
    }

    // 6. a hold at a rapid rate still stops
    {
        Run r = hold_from(200.0, 1000.0, 10000000, 1);
        CHECK(r.stopped);
        printf("200 mm/s: %.1f ms, %.3f mm\n", r.ticks / TICK_HZ * 1000, r.mm);
    }

    // 7. the feed rate the hold was tested at on the machine
    {
        Run r = hold_from(10.0, 1000.0, 1000000, 1);
        CHECK(r.stopped);
        double expect_s = 10.0 / 1000.0;
        printf("600 mm/min: %.1f ms (expect %.1f), %.3f mm (expect %.3f)\n",
               r.ticks / TICK_HZ * 1000, expect_s * 1000, r.mm, (10.0*10.0)/(2*1000.0));
        CHECK(fabs(r.ticks / TICK_HZ - expect_s) < expect_s * 0.05);
    }

    // 8. a hold in an accelerating block: the cap has to win, or the block's own ramp cancels
    // the braking and the path runs on at speed until it is cut off dead
    {
        const int64_t accel = accel_to_fp(1000.0 * STEPS_PER_MM);
        const int64_t decel = accel;
        const int64_t plateau = rate_to_fp(100.0 * STEPS_PER_MM);
        int64_t spt = rate_to_fp(5.0 * STEPS_PER_MM);
        int64_t hold_rate = spt;

        double start = 0, peak = 0;
        uint32_t ticks = 0;
        while(hold_rate > 0 && ticks < 2000000) {
            ticks++;
            hold_rate = hold_rate > decel ? hold_rate - decel : 0;
            spt += accel;
            if(spt > plateau) spt = plateau;
            int64_t use = hold_rate < spt ? hold_rate : spt;
            double v = (double)use / STEPTICKER_FPSCALE * TICK_HZ / STEPS_PER_MM;
            if(ticks == 1) start = v;
            if(v > peak) peak = v;
        }
        double got = ticks / TICK_HZ, expect = 5.0 / 1000.0;
        printf("hold in an accelerating block: %.2f ms (expect %.2f), peak %.3f of %.3f mm/s\n",
               got * 1000, expect * 1000, peak, start);
        CHECK(peak <= start * 1.01);          // it must never speed up
        CHECK(fabs(got - expect) < expect * 0.05);
    }

    // 9. the motors follow the path: the path axis steps as its counter crosses a step, the others
    // sit within half a step of the line through it, every step interval is within one tick of
    // even, and each motor lands on its last step with the path's last step, held or not
    {
        const uint32_t steps_of[MOTORS] = {113, 61, 9};
        Ramp ramp;
        ramp.steps_per_tick = rate_to_fp(50.0 * STEPS_PER_MM);
        ramp.deceleration_change = -accel_to_fp(10000.0 * STEPS_PER_MM);   // stops inside the block

        for(int held = 0; held < 2; held++) {
            Ticker t;
            t.start(ramp, steps_of);
            int64_t hold_rate = t.steps_per_tick;
            double worst_off = 0, worst_jitter = 0;
            uint32_t last_step_tick[MOTORS] = {0, 0, 0}, last_gap[MOTORS] = {0, 0, 0};
            uint32_t ticks = 0;
            bool moving = true;
            while(moving && ticks < 1000000) {
                ticks++;
                int64_t use = t.steps_per_tick;
                if(held && ticks > 40) {   // a hold part way, the cap never lifted
                    int64_t a = -ramp.deceleration_change;
                    hold_rate = hold_rate > a ? hold_rate - a : 0;
                    if(hold_rate == 0) break;
                    use = hold_rate;
                }
                uint32_t before[MOTORS];
                for(int i = 0; i < MOTORS; i++) before[i] = t.m[i].step_count;
                moving = t.tick(use);
                double pos = t.step_count + (double)t.counter / STEPTICKER_FPSCALE;
                CHECK(pos - t.m[0].step_count >= 0 && pos - t.m[0].step_count < 1.0);
                for(int i = 1; i < MOTORS; i++) {
                    double off = fabs(t.m[i].step_count - pos * steps_of[i] / steps_of[0]);
                    if(off > worst_off) worst_off = off;
                }
                for(int i = 0; i < MOTORS; i++) {
                    if(t.m[i].step_count == before[i]) continue;
                    if(last_step_tick[i] != 0) {
                        uint32_t gap = ticks - last_step_tick[i];
                        if(!held && last_gap[i] != 0) {
                            double jitter = fabs((double)gap - last_gap[i]);
                            if(jitter > worst_jitter) worst_jitter = jitter;
                        }
                        last_gap[i] = gap;
                    }
                    last_step_tick[i] = ticks;
                }
            }
            printf("%s: worst %.3f steps off the line, step intervals jitter %.0f tick(s), "
                   "motors at %u/%u/%u\n", held ? "held" : "run", worst_off, worst_jitter,
                   t.m[0].step_count, t.m[1].step_count, t.m[2].step_count);
            CHECK(worst_off <= 0.5 + 1e-6);
            if(!held) {
                CHECK(worst_jitter <= 1.0);
                for(int i = 0; i < MOTORS; i++) CHECK(t.m[i].step_count == steps_of[i]);
            } else {
                CHECK(t.m[0].step_count < steps_of[0]);   // it did stop short
            }
        }
    }

    printf(fails ? "%d failures\n" : "all passed\n", fails);
    return fails != 0;
}
