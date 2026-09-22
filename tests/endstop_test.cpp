// Endstops::service is only the limit check now: homing terminates its own moves through the step
// ticker's watch, which watch_test covers. What is left is when a limit halts the machine and when
// the latch clears.
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

enum STATUS { NOT_HOMING, HOMING, BACK_OFF_HOME, LIMIT_TRIGGERED };
enum { HARD_LIMIT = 21, MOTOR_ERROR_X = 30 };

struct endstop_info_t {
    bool     pressed;
    uint16_t debounce;
    char     axis;
    uint8_t  axis_index;
    bool     limit_enable;
    bool     at_end;
    bool     at_max;
};

static std::vector<endstop_info_t *> endstops;
static std::vector<endstop_info_t *> motor_alarms;
static bool     motor_moving[6];
static bool     motor_negative[6];
static int32_t  motor_pos[6];
static STATUS   status;
static uint32_t limit_clear_ms;
static const uint32_t LIMIT_RELEASE_MS = 100;
static bool     halted;
static uint8_t  halt_reason;
static std::vector<int> stopped;

static void halt(uint8_t reason) { if(!halted) halt_reason = reason; halted = true; }

static bool limit_tripped;      // the step ticker's flag
static uint16_t limit_hysteresis;
static bool     limit_seen;
static endstop_info_t *limit_held;
static int32_t  limit_at_step;
static endstop_info_t *suspended;   // the switch homing is approaching, if any

// StepTicker::check_limits, with step_tick's is_halted return folded in
static void check_limits()
{
    if(halted || limit_tripped) return;

    if(limit_seen && !limit_held->pressed) limit_seen = false;

    endstop_info_t *closing = nullptr;
    for(auto& l : endstops) {
        if(!l->limit_enable || l == suspended) continue;
        if(!motor_moving[l->axis_index] || !l->pressed) continue;
        if(l->at_end && motor_negative[l->axis_index] == l->at_max) continue;
        closing = l;
        break;
    }
    if(closing == nullptr) return;

    if(!limit_seen || closing != limit_held) {
        limit_seen = true;
        limit_held = closing;
        limit_at_step = motor_pos[closing->axis_index];
    }
    int32_t into = motor_pos[limit_held->axis_index] - limit_at_step;
    if(!limit_held->at_end) {
        into = abs(into);
    } else if(!limit_held->at_max) {
        into = -into;
    }

    if(into < limit_hysteresis)
        return;

    for (int m = 0; m < 6; m++) motor_moving[m] = false;
    limit_tripped = true;
    halt(HARD_LIMIT);
}

static void check_motor_alarms()
{
    if(halted) return;
    for(auto& i : motor_alarms) {
        if(i->pressed) { halt(MOTOR_ERROR_X + i->axis_index); return; }
    }
}

// Endstops::service, verbatim
static void service()
{
    check_motor_alarms();

    if(status == LIMIT_TRIGGERED) {
        for(auto& i : endstops) {
            if(i->limit_enable && i->pressed) { limit_clear_ms = 0; return; }
        }
        if(limit_clear_ms++ >= LIMIT_RELEASE_MS) {
            status = NOT_HOMING;
            limit_tripped = false; limit_seen = false;
        }
        return;
    }

    if(!limit_tripped) return;
    status = LIMIT_TRIGGERED;
}

static endstop_info_t *mk(char axis, uint8_t idx, bool limit, bool at_end = true, bool at_max = true)
{
    auto *e = new endstop_info_t{false, 0, axis, idx, limit, at_end, at_max};
    endstops.push_back(e);
    return e;
}

static endstop_info_t *xmax, *ymax, *zmax, *amax;
static void reset()
{
    for(auto *e : endstops) delete e;
    for(auto *e : motor_alarms) delete e;
    endstops.clear(); motor_alarms.clear();
    for(int i = 0; i < 6; i++) { motor_moving[i] = false; motor_negative[i] = false; motor_pos[i] = 0; }
    status = NOT_HOMING; limit_clear_ms = 0;
    halted = false; halt_reason = 0; stopped.clear();
    limit_tripped = false; limit_seen = false; limit_hysteresis = 0; suspended = nullptr;

    xmax = mk('X', 0, true);
    ymax = mk('Y', 1, true);
    zmax = mk('Z', 2, true);
    amax = mk('A', 3, false);   // a rotary flag: homes to it, never a limit
}

// one step per tick for every moving motor
static void ticks(int n)
{
    for(int i = 0; i < n; i++) {
        check_limits();
        service();
        for(int m = 0; m < 6; m++) if(motor_moving[m]) motor_pos[m] += motor_negative[m] ? -1 : 1;
    }
}

int main()
{
    // a limit closing on a moving axis halts
    reset();
    motor_moving[2] = true;
    zmax->pressed = true;
    ticks(2);
    CHECK(halted && halt_reason == HARD_LIMIT);
    CHECK(status == LIMIT_TRIGGERED);
    CHECK(!motor_moving[2]);

    // the axis must travel limit_hysteresis steps with the switch held
    reset();
    limit_hysteresis = 3;
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(3);                            // latched at 0, stepped to 2
    CHECK(!halted);
    ticks(1);                            // at 3
    CHECK(halted);

    // a glitch that does not persist resets the distance
    reset();
    limit_hysteresis = 3;
    motor_moving[0] = true;
    xmax->pressed = true;  ticks(3);
    xmax->pressed = false; ticks(1);
    xmax->pressed = true;  ticks(3);
    CHECK(!halted);

    // a still axis on a switch never trips, however long it sits there
    reset();
    limit_hysteresis = 3;
    motor_moving[0] = false;
    xmax->pressed = true;
    ticks(500);
    CHECK(!halted);

    // the distance into a held switch adds up across blocks: pecking at it does not slip through
    reset();
    limit_hysteresis = 10;
    xmax->pressed = true;
    for(int peck = 0; peck < 2; peck++) {
        motor_moving[0] = true;  ticks(4);   // 4 steps in
        motor_moving[0] = false; ticks(5);   // another axis works, X sits on the switch
    }
    CHECK(!halted);                          // 8 in so far
    motor_moving[0] = true; ticks(3);        // 8, 9, 10
    CHECK(halted && halt_reason == HARD_LIMIT);

    // backing off while still pressed counts against the distance, not toward it
    reset();
    limit_hysteresis = 10;
    motor_moving[0] = true; xmax->pressed = true;
    ticks(1);                                 // latched at 0, now at 1
    motor_negative[0] = true;  ticks(6);      // retreat to -5, exempt while moving away
    motor_negative[0] = false; ticks(15);     // -5 .. 9 seen
    CHECK(!halted);
    ticks(1);                                 // 10 in
    CHECK(halted);

    // a second switch closing on another axis is measured on that axis
    reset();
    limit_hysteresis = 3;
    motor_moving[0] = true; xmax->pressed = true;
    ticks(2);                                 // X latched, 2 in, then its block ends
    motor_moving[0] = false;
    motor_moving[1] = true; ymax->pressed = true;
    ticks(3);                                 // Y latched fresh: 0, 1, 2
    CHECK(!halted);
    ticks(1);
    CHECK(halted);

    // a switch under a still axis is not a limit
    reset();
    motor_moving[2] = false;
    zmax->pressed = true;
    ticks(50);
    CHECK(!halted);

    // leaving a switch is how you get off it, so moving away never halts
    reset();
    motor_moving[0] = true; motor_negative[0] = true;   // max switch, moving away
    xmax->pressed = true;
    ticks(50);
    CHECK(!halted);
    motor_negative[0] = false;                          // now closing on it
    ticks(2);
    CHECK(halted && halt_reason == HARD_LIMIT);

    // the switch being approached is the step ticker's business, not this one
    reset();
    status = HOMING;
    suspended = xmax;
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(50);
    CHECK(!halted);
    CHECK(!limit_tripped);

    // ... including the retract off that switch while it is still held
    reset();
    status = BACK_OFF_HOME;
    suspended = xmax;
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(50);
    CHECK(!halted);

    // but another axis hitting its limit during homing still halts
    reset();
    status = HOMING;
    suspended = xmax;                 // X is the one being homed
    motor_moving[1] = true;
    ymax->pressed = true;               // Y should not be anywhere near its switch
    ticks(2);
    CHECK(halted && halt_reason == HARD_LIMIT);

    // a switch with limit_enable false never halts, whatever it does
    reset();
    motor_moving[3] = true;
    amax->pressed = true;
    ticks(50);
    CHECK(!halted);
    CHECK(!limit_tripped);

    // the latch clears only when every limit has been released for LIMIT_RELEASE_MS
    reset();
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(status == LIMIT_TRIGGERED);
    zmax->pressed = true;
    xmax->pressed = false;
    ticks(LIMIT_RELEASE_MS + 10);
    CHECK(status == LIMIT_TRIGGERED);
    zmax->pressed = false;
    ticks(LIMIT_RELEASE_MS + 1);
    CHECK(status == NOT_HOMING);

    // one triggered switch does not re-stop the axis every tick
    reset();
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(limit_tripped);
    halted = false;                      // as if unlocked, switch still held
    status = NOT_HOMING;
    motor_moving[0] = true;
    ticks(20);
    CHECK(limit_tripped);

    // a motor alarm halts with the axis in the reason, with no debounce, and with every
    // endstop clear: the fast path has to cover the alarm pins or a driver fault is invisible
    reset();
    auto *alarm = new endstop_info_t{true, 0, 'Y', 1, false, true, true};
    motor_alarms.push_back(alarm);
    for(auto *e : endstops) e->pressed = false;
    ticks(1);
    CHECK(halted && halt_reason == MOTOR_ERROR_X + 1);

    // a limit is not detected while halted
    reset();
    halted = true;
    motor_moving[0] = true; xmax->pressed = true;
    ticks(50);
    CHECK(!limit_tripped);

    // an alarm is seen even while a limit is latched: two independent faults
    reset();
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(status == LIMIT_TRIGGERED);
    halted = false; halt_reason = 0;          // as if unlocked, limit still held
    auto *a2 = new endstop_info_t{true, 0, 'Z', 2, false, true, true};
    motor_alarms.push_back(a2);
    ticks(1);
    CHECK(halted && halt_reason == MOTOR_ERROR_X + 2);

    // a mid-travel limit is reached from either side, so direction does not excuse it
    reset();
    auto *mid = mk('X', 0, true, false);   // limit, not at an end
    motor_moving[0] = true; motor_negative[0] = true;
    mid->pressed = true;
    ticks(2);
    CHECK(limit_tripped);

    reset();
    mid = mk('X', 0, true, false);
    motor_moving[0] = true; motor_negative[0] = false;   // the other way, still a limit
    mid->pressed = true;
    ticks(2);
    CHECK(limit_tripped);

    printf(fails == 0 ? "endstops: all passed\n" : "endstops: %d FAILED\n", fails);
    return fails != 0;
}
