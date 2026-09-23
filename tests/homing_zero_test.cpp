// Where homing puts the zero: at the switch edge, not where the axis came to rest.
// c++ -std=c++11 homing_zero_test.cpp
#include <cstdio>
#include <cstdlib>
#include <cmath>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

static const float steps_per_mm = 160.0f;
static const float hysteresis_mm = 0.1f;

// One axis: steps counted by the ticker, a switch that closes at a fixed step.
struct Axis {
    int32_t pos;                  // current_position_steps
    int32_t switch_at;            // the step where the switch first reads closed
    bool    to_max;               // homing direction: true = travels positive
    int32_t at_steps;             // what the watch latched
    bool    hit;
};

// StepTicker::check_watch plus the stepping loop, for one axis
static void approach(Axis &a, float distance_mm)
{
    a.hit = false;
    int32_t steps = (int32_t)(distance_mm * steps_per_mm);
    int32_t hyst = (int32_t)(hysteresis_mm * steps_per_mm);
    bool seen = false;
    for (int32_t i = 0; i < steps && !a.hit; i++) {
        bool closed = a.to_max ? (a.pos >= a.switch_at) : (a.pos <= a.switch_at);
        if(!closed) {
            seen = false;
        } else {
            if(!seen) { seen = true; a.at_steps = a.pos; }
            if(abs(a.pos - a.at_steps) >= hyst) { a.hit = true; break; }
        }
        a.pos += a.to_max ? 1 : -1;
    }
}

static void retract(Axis &a, float mm)
{
    a.pos += (int32_t)(mm * steps_per_mm) * (a.to_max ? -1 : 1);
}

// Endstops::past_edge_mm
static float past_edge_mm(const Axis &a) { return (a.pos - a.at_steps) / steps_per_mm; }

// the machine position the axis reports once homing assigns it
static float reported_position(const Axis &a, float homing_position)
{
    return homing_position + past_edge_mm(a);
}

// where the switch edge is, in that same frame
static float edge_position(const Axis &a, float reported)
{
    return reported - (a.pos - a.at_steps) / steps_per_mm;
}

int main()
{
    // homing to max: the edge is at machine position 0 whatever the overshoot
    {
        Axis a{ -50 * (int32_t)steps_per_mm, 0, true, 0, false };
        approach(a, 500);                       // fast pass
        CHECK(a.hit);
        retract(a, 1);
        approach(a, 2);                         // slow pass
        CHECK(a.hit);
        float reported = reported_position(a, 0.0f);
        CHECK(fabsf(edge_position(a, reported) - 0.0f) < 1e-4f);
        CHECK(reported > 0.0f);                 // the axis stands past the edge
        CHECK(fabsf(reported - hysteresis_mm) < 0.01f);
    }

    // homing to min: the same, mirrored
    {
        Axis a{ 50 * (int32_t)steps_per_mm, 0, false, 0, false };
        approach(a, 500);
        CHECK(a.hit);
        retract(a, 1);
        approach(a, 2);
        CHECK(a.hit);
        float reported = reported_position(a, 0.0f);
        CHECK(fabsf(edge_position(a, reported) - 0.0f) < 1e-4f);
        CHECK(reported < 0.0f);
        CHECK(fabsf(reported + hysteresis_mm) < 0.01f);
    }

    // a non-zero homing position just shifts the frame
    {
        Axis a{ -50 * (int32_t)steps_per_mm, 0, true, 0, false };
        approach(a, 500);
        retract(a, 1);
        approach(a, 2);
        float reported = reported_position(a, -1.0f);
        CHECK(fabsf(edge_position(a, reported) + 1.0f) < 1e-4f);
    }

    // the zero does not move with the approach speed: a slower pass overshoots the same distance,
    // because the distance is counted in steps and not in ticks
    {
        Axis fast{ -50 * (int32_t)steps_per_mm, 0, true, 0, false };
        approach(fast, 500); retract(fast, 1); approach(fast, 2);
        Axis slow{ -50 * (int32_t)steps_per_mm, 0, true, 0, false };
        approach(slow, 500); retract(slow, 1); approach(slow, 2);
        CHECK(reported_position(fast, 0.0f) == reported_position(slow, 0.0f));
    }

    // and it does not move when the switch sits somewhere else
    {
        Axis a{ -50 * (int32_t)steps_per_mm, -3 * (int32_t)steps_per_mm, true, 0, false };
        approach(a, 500); retract(a, 1); approach(a, 2);
        float reported = reported_position(a, 0.0f);
        CHECK(fabsf(edge_position(a, reported) - 0.0f) < 1e-4f);
    }

    printf(fails == 0 ? "homing zero: all passed\n" : "homing zero: %d FAILED\n", fails);
    return fails != 0;
}
