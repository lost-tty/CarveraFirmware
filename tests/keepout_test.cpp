// Host test of the keep-out box: segment against box with open sides.
// c++ -std=c++11 -I../src/libs keepout_test.cpp
#include "KeepOut.h"
#include <cstdio>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

int main()
{
    // the tool rack: X -29..0, any Y, Z -30 and below
    KeepOut rack;
    rack.min[0] = -29; rack.max[0] = 0;
    rack.max[2] = -30;

    // a rapid across the rack at working depth, both ends clear of it
    { float a[3]{-100, -50, -40}, b[3]{ 20, -50, -40}; CHECK(rack.crossed(a, b)); }
    // the same crossing above the rack
    { float a[3]{-100, -50, -20}, b[3]{ 20, -50, -20}; CHECK(!rack.crossed(a, b)); }
    // descending into it from above
    { float a[3]{-10, -50, -10}, b[3]{-10, -50, -60}; CHECK(rack.crossed(a, b)); }
    // descending beside it
    { float a[3]{-40, -50, -10}, b[3]{-40, -50, -60}; CHECK(!rack.crossed(a, b)); }
    // a diagonal that dips under the top only while over the rack
    { float a[3]{-60, -50, -25}, b[3]{ 30, -50, -35}; CHECK(rack.crossed(a, b)); }
    // a diagonal that passes over: below the top only once past X 0
    { float a[3]{-60, -50, -20}, b[3]{ 40, -50, -35}; CHECK(!rack.crossed(a, b)); }
    // ending exactly on the boundary counts
    { float a[3]{-100, -50, -40}, b[3]{-29, -50, -40}; CHECK(rack.crossed(a, b)); }
    // a zero-length move outside and inside
    { float a[3]{-50, -50, -40}; CHECK(!rack.crossed(a, a)); }
    { float a[3]{-10, -50, -40}; CHECK(rack.crossed(a, a)); CHECK(rack.contains(a)); }
    // Y is open: any Y is over the rack
    { float a[3]{-10, 500, -40}, b[3]{-10, -900, -40}; CHECK(rack.crossed(a, b)); }
    // straight up out of the rack still crosses it; the caller decides to allow that
    { float a[3]{-10, -50, -40}, b[3]{-10, -50, -10}; CHECK(rack.crossed(a, b)); CHECK(!rack.contains(b)); }

    // a zone with no bound at all is no zone
    { KeepOut none; CHECK(none.unbounded()); CHECK(!rack.unbounded()); }

    // a fully bounded box behaves the same on the bounded axis
    KeepOut box;
    box.min[1] = -100; box.max[1] = -50;
    { float a[3]{0, -200, 0}, b[3]{0, 0, 0}; CHECK(box.crossed(a, b)); }
    { float a[3]{0, -40, 0}, b[3]{0, 0, 0}; CHECK(!box.crossed(a, b)); }

    printf(fails == 0 ? "keepout: all passed\n" : "keepout: %d FAILED\n", fails);
    return fails != 0;
}
