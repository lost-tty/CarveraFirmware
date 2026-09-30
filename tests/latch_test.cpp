// The latch: where an input asserted, as the step counters at the first sample that saw it.
//
// What has to hold, or a probe measures the wrong point or a homing zero drifts:
//   - nothing is latched while the input is clear
//   - the first asserted sample latches the counters of that sample, exactly
//   - the hit waits for the hysteresis, and the latch does not move while it waits
//   - an input that drops before the hysteresis starts over at its next edge
//   - only watched motors count towards the hysteresis, in either direction
//   - a hit triggers the latch and leaves it pending; arming clears that
//
// c++ -std=c++11 -I ../src/libs -I ../src/modules/robot latch_test.cpp
#include "ProbeLatch.h"

#include <cstdio>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

static const uint8_t N = 3;

int main()
{
    ProbeLatch latch;
    WatchRule w;

    // 1. clear input: no latch, no hit
    {
        w.motors = 1; w.hysteresis = 0; w.arm(); latch.arm();
        int32_t now[N] = {100, 200, 300};
        CHECK(!w.sample(false, now, N, latch));
        CHECK(!w.seen && !w.hit && latch.state == ProbeLatch::ARMED && !latch.pending);
        printf("clear: ok\n");
    }

    // 2. a probe: hysteresis 0, the first edge is the hit, the latch is that sample
    {
        w.motors = 1 << 2; w.hysteresis = 0; w.arm(); latch.arm();
        int32_t now[N] = {100, 200, 300};
        CHECK(!w.sample(false, now, N, latch));
        now[2] = 299;
        CHECK(w.sample(true, now, N, latch));
        CHECK(w.hit && latch.steps[2] == 299 && latch.steps[0] == 100);
        CHECK(latch.state == ProbeLatch::TRIGGERED && latch.pending);
        printf("probe: ok\n");
    }

    // 3. hysteresis 5 on motor 0: the latch stays at the edge, the hit comes 5 steps later
    {
        w.motors = 1; w.hysteresis = 5; w.arm(); latch.arm();
        int32_t now[N] = {1000, 0, 0};
        for (int32_t x = 1000; x < 1005; x++) {
            now[0] = x;
            CHECK(!w.sample(true, now, N, latch));
            CHECK(latch.steps[0] == 1000);
            CHECK(!latch.pending);
        }
        now[0] = 1005;
        CHECK(w.sample(true, now, N, latch));
        CHECK(latch.steps[0] == 1000 && latch.pending);
        printf("hysteresis: ok\n");
    }

    // 4. a bounce: released before the hysteresis, the next edge latches afresh
    {
        w.motors = 1; w.hysteresis = 5; w.arm(); latch.arm();
        int32_t now[N] = {50, 0, 0};
        CHECK(!w.sample(true, now, N, latch));
        now[0] = 52;
        CHECK(!w.sample(false, now, N, latch));
        CHECK(!w.seen);
        now[0] = 53;
        CHECK(!w.sample(true, now, N, latch));
        CHECK(latch.steps[0] == 53);
        now[0] = 57;
        CHECK(!w.sample(true, now, N, latch));
        now[0] = 58;
        CHECK(w.sample(true, now, N, latch));
        printf("bounce: ok\n");
    }

    // 5. only watched motors count, and travel counts either way
    {
        w.motors = 1 << 1; w.hysteresis = 3; w.arm(); latch.arm();
        int32_t now[N] = {0, 500, 0};
        CHECK(!w.sample(true, now, N, latch));
        now[0] = 100;   // motor 0 runs away: not watched
        CHECK(!w.sample(true, now, N, latch));
        now[1] = 497;   // motor 1 backwards by 3
        CHECK(w.sample(true, now, N, latch));
        CHECK(latch.steps[1] == 500);
        printf("motors: ok\n");
    }

    // 6. arming clears a pending trigger; clear() stands the latch down
    {
        CHECK(latch.pending);
        latch.arm();
        CHECK(!latch.pending && latch.state == ProbeLatch::ARMED);
        latch.clear();
        CHECK(latch.state == ProbeLatch::IDLE);
        printf("arm/clear: ok\n");
    }

    if (fails == 0) printf("all ok\n");
    return fails == 0 ? 0 : 1;
}
