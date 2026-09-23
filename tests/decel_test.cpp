// The ramp a motor follows when something tells it to stop mid-block: does 2.62 fixed point have
// the resolution at slow approach rates, and does the distance match v^2/2a?
// c++ -std=c++11 decel_test.cpp
#include <cstdio>
#include <cstdint>
#include <cmath>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

#define STEPTICKER_FPSCALE (1LL<<62)
static const double TICK_HZ = 100000.0;
static const double STEPS_PER_MM = 200.0;

// Block::prepare's conversion, for one motor
static int64_t rate_to_fp(double steps_per_sec)
{
    return (int64_t)round((steps_per_sec / TICK_HZ) * STEPTICKER_FPSCALE);
}

// acceleration in steps/s^2 to the per-tick increment, as Block::fp_scale does it
static int64_t accel_to_fp(double steps_per_s2)
{
    double fp_scale = (double)STEPTICKER_FPSCALE / (TICK_HZ * TICK_HZ);
    return (int64_t)round(steps_per_s2 * fp_scale);
}

// StepTicker::step_tick's stopping branch, run until the motor is done
struct Result { double mm; uint32_t ticks; };
static Result decelerate(double from_mm_s, double accel_mm_s2)
{
    int64_t rate  = rate_to_fp(from_mm_s * STEPS_PER_MM);
    int64_t decel = accel_to_fp(accel_mm_s2 * STEPS_PER_MM);

    int64_t counter = 0;
    uint32_t steps = 0, ticks = 0;
    while(rate > 0) {
        rate -= decel;
        if(rate < 0) rate = 0;
        counter += rate;
        if(counter >= STEPTICKER_FPSCALE) {
            counter -= STEPTICKER_FPSCALE;
            steps++;
        }
        ticks++;
        if(ticks > 10000000) break;   // no ramp should take a hundred seconds
    }
    return Result{steps / STEPS_PER_MM, ticks};
}

int main()
{
    // the distance is v^2/2a, whatever the rate
    struct { double v, a; } cases[]= {
        {25.0, 150.0},   // the fast homing approach
        {15.0, 150.0},
        {3.0,  150.0},   // the slow approach, where the per-tick numbers are smallest
        {1.0,  150.0},
        {0.5,  150.0},
        {100.0, 150.0},  // a rapid
        {3.0,  500.0},   // Z, which has its own acceleration
    };

    for(auto &c : cases) {
        Result r = decelerate(c.v, c.a);
        double expect = (c.v * c.v) / (2 * c.a);
        double err = fabs(r.mm - expect);
        printf("%6.1f mm/s at %5.1f mm/s^2: %8.4f mm in %7u ticks (%.4f expected, err %.4f)\n",
               c.v, c.a, r.mm, r.ticks, expect, err);
        // within a step, or 2%, whichever is larger: the step quantum is 5 um here
        CHECK(err <= fmax(1.0 / STEPS_PER_MM, expect * 0.02));
        CHECK(r.ticks < 10000000);
    }

    // the slow approach must not round to a ramp that never ends
    {
        int64_t decel = accel_to_fp(150.0 * STEPS_PER_MM);
        printf("decel_per_tick at 150 mm/s^2 = %lld (fpscale %lld)\n", (long long)decel, (long long)STEPTICKER_FPSCALE);
        CHECK(decel > 0);
    }

    // and the smallest acceleration anyone might configure still gives a non-zero step
    {
        int64_t decel = accel_to_fp(1.0 * STEPS_PER_MM);
        printf("decel_per_tick at 1 mm/s^2 = %lld\n", (long long)decel);
        CHECK(decel > 0);
    }

    // the scale must be live before a ramp is computed: with fp_scale still zero, as it is before
    // Block::init runs, the decrement is zero and the motor never stops
    {
        double unset_scale = 0;
        int64_t decel = (int64_t)round(150.0 * STEPS_PER_MM * unset_scale);
        CHECK(decel == 0);                      // what a stale scale gives

        int64_t rate = rate_to_fp(25.0 * STEPS_PER_MM);
        uint32_t ticks = 0;
        while(rate > 0 && ticks < 1000) { rate -= decel; ticks++; }
        CHECK(rate > 0);                        // it never reaches zero, so the axis keeps going
    }

    // stopping from rest is a no-op, not a hang
    {
        Result r = decelerate(0.0, 150.0);
        CHECK(r.mm == 0.0);
        CHECK(r.ticks <= 1);
    }

    printf(fails == 0 ? "decel: all passed\n" : "decel: %d FAILED\n", fails);
    return fails != 0;
}
