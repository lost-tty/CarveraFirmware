// Endstops::service decides, for every configured switch, whether a trigger means "homed" or
// "crash". The decision is transcribed from Endstops.cpp; the pins, the robot and the halt are
// faked so a whole homing cycle can be driven tick by tick.
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

enum STATUS { NOT_HOMING, MOVING_TO_ENDSTOP_FAST, MOVING_TO_ENDSTOP_SLOW, MOVING_BACK, LIMIT_TRIGGERED };
enum { HARD_LIMIT = 21, MOTOR_ERROR_X = 30 };

struct endstop_info_t {
    bool     pressed;        // stands in for pin.get()
    uint16_t debounce;
    bool     triggered;
    char     axis;
    uint8_t  axis_index;
    bool     limit_enable;
};

// ---- the world service() talks to -------------------------------------------------------------
static std::vector<endstop_info_t *> endstops;
static std::vector<endstop_info_t *> motor_alarms;
static endstop_info_t *homing_pin[6];      // homing_axis[m].pin_info
static bool     axis_to_home[6];
static bool     motor_moving[6];
static STATUS   status;
static uint32_t debounce_ms;
static uint32_t limit_clear_ms;
static const uint32_t LIMIT_RELEASE_MS = 100;
static bool     halted;
static uint8_t  halt_reason;
static std::vector<int> stopped;           // axes stop_motor was called on

static void halt(uint8_t reason) { if(!halted) halt_reason = reason; halted = true; }
static void stop_motor(uint8_t m) { stopped.push_back(m); motor_moving[m] = false; }

// ---- Endstops::homing_toward, verbatim --------------------------------------------------------
static bool homing_toward(const endstop_info_t *e)
{
    if(status == NOT_HOMING || status == LIMIT_TRIGGERED) return false;
    uint8_t m = e->axis_index;
    if(m >= 6) return false;
    return axis_to_home[m] && homing_pin[m] == e;
}

// ---- Endstops::service, verbatim --------------------------------------------------------------
static void service()
{
    if(status == LIMIT_TRIGGERED) {
        for(auto& i : endstops) {
            if(i->limit_enable && i->pressed) { limit_clear_ms = 0; return; }
        }
        if(limit_clear_ms++ >= LIMIT_RELEASE_MS) status = NOT_HOMING;
        return;
    }

    if(halted) return;

    for(auto& i : endstops) {
        if(!i->limit_enable && !homing_toward(i)) continue;

        if(!i->pressed) { i->debounce = 0; continue; }   // the press ends at the pin, not the motor
        if(i->debounce > debounce_ms) continue;          // already acted on this one
        if(!motor_moving[i->axis_index]) continue;
        if(++i->debounce < debounce_ms) continue;
        i->debounce= debounce_ms + 1;

        stop_motor(i->axis_index);

        if(homing_toward(i)) {
            if(status == MOVING_TO_ENDSTOP_FAST || status == MOVING_TO_ENDSTOP_SLOW) i->triggered = true;
        } else {
            status = LIMIT_TRIGGERED;
            halt(HARD_LIMIT);
            return;
        }
    }

    for(auto& i : motor_alarms) {
        if(i->pressed) { halt(MOTOR_ERROR_X + i->axis_index); return; }
    }
}

// Endstops::back_off_home's selection rule: step off any switch we homed onto
static bool backs_off(const endstop_info_t *e) { return e != nullptr && e->triggered; }

// ---- fixture ----------------------------------------------------------------------------------
static endstop_info_t *mk(char axis, uint8_t idx, bool limit)
{
    auto *e = new endstop_info_t{false, 0, false, axis, idx, limit};
    endstops.push_back(e);
    return e;
}

// X/Y/Z as shipped: each homes to its max switch and that same switch is a limit
static endstop_info_t *xmax, *ymax, *zmax, *amax;
static void reset(bool a_axis_limit = false)
{
    for(auto *e : endstops) delete e;
    for(auto *e : motor_alarms) delete e;
    endstops.clear(); motor_alarms.clear();
    for(int i = 0; i < 6; i++) { homing_pin[i] = nullptr; axis_to_home[i] = false; motor_moving[i] = false; }
    status = NOT_HOMING; debounce_ms = 1; limit_clear_ms = 0;
    halted = false; halt_reason = 0; stopped.clear();

    xmax = mk('X', 0, true);  homing_pin[0] = xmax;
    ymax = mk('Y', 1, true);  homing_pin[1] = ymax;
    zmax = mk('Z', 2, true);  homing_pin[2] = zmax;
    // a rotary axis: homes to a flag that is not a limit
    amax = mk('A', 3, a_axis_limit); homing_pin[3] = amax;
}

static void ticks(int n) { for(int i = 0; i < n; i++) service(); }

int main()
{
    // a switch must be held for debounce_ms consecutive ticks
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true; motor_moving[0] = true;
    debounce_ms = 3;
    xmax->pressed = true;
    ticks(2);
    CHECK(stopped.empty());
    CHECK(!xmax->triggered);
    ticks(1);
    CHECK(stopped.size() == 1 && stopped[0] == 0);
    CHECK(xmax->triggered);

    // a glitch that does not persist resets the count
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true; motor_moving[0] = true;
    debounce_ms = 3;
    xmax->pressed = true;  ticks(2);
    xmax->pressed = false; ticks(1);
    xmax->pressed = true;  ticks(2);
    CHECK(stopped.empty());

    // homing X: hitting X's own switch is expected, not a limit
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true; motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(xmax->triggered);
    CHECK(!halted);

    // the retract runs while still on the switch and must not halt
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true; motor_moving[0] = true;
    xmax->pressed = true; ticks(2);
    CHECK(xmax->triggered);
    status = MOVING_BACK; motor_moving[0] = true;     // retracting, switch still pressed
    ticks(10);
    CHECK(!halted);

    // ... and the slow re-approach re-arms triggered (home() clears the latch between passes)
    xmax->triggered = false; xmax->debounce = 0;
    status = MOVING_TO_ENDSTOP_SLOW; motor_moving[0] = true;
    ticks(2);
    CHECK(xmax->triggered);
    CHECK(!halted);

    // homing X while Y hits its limit is a crash, not a home
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true;
    motor_moving[0] = true; motor_moving[1] = true;
    ymax->pressed = true;
    ticks(2);
    CHECK(halted);
    CHECK(halt_reason == HARD_LIMIT);
    CHECK(status == LIMIT_TRIGGERED);
    CHECK(!ymax->triggered);

    // a limit during normal motion halts
    reset();
    motor_moving[2] = true;
    zmax->pressed = true;
    ticks(2);
    CHECK(halted && halt_reason == HARD_LIMIT);

    // a switch that is not pressed while the axis is still is ignored
    reset();
    motor_moving[2] = false;
    zmax->pressed = true;
    ticks(50);
    CHECK(!halted);

    // a rotary home flag with limit_enable false never halts, but does stop the axis when homing
    reset();
    motor_moving[3] = true;
    amax->pressed = true;
    ticks(50);
    CHECK(!halted);                       // not a limit
    CHECK(stopped.empty());
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[3] = true; motor_moving[3] = true;
    ticks(2);
    CHECK(stopped.size() == 1 && stopped[0] == 3);
    CHECK(amax->triggered);

    // the latch clears only when every limit has been released for LIMIT_RELEASE_MS
    reset();
    motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(status == LIMIT_TRIGGERED);
    zmax->pressed = true;                 // a second switch still held
    xmax->pressed = false;
    ticks(LIMIT_RELEASE_MS + 10);
    CHECK(status == LIMIT_TRIGGERED);     // must not clear while Z is pressed
    zmax->pressed = false;
    ticks(LIMIT_RELEASE_MS + 1);
    CHECK(status == NOT_HOMING);

    // one triggered switch does not re-stop the axis every tick
    reset();
    status = MOVING_TO_ENDSTOP_FAST; axis_to_home[0] = true; motor_moving[0] = true;
    xmax->pressed = true;
    ticks(2);
    CHECK(stopped.size() == 1);
    motor_moving[0] = true;               // pretend it is still coasting
    ticks(20);
    CHECK(stopped.size() == 1);

    // a rotary flag is backed off after homing like any other axis, limit or not
    reset();
    status = MOVING_TO_ENDSTOP_SLOW; axis_to_home[3] = true; motor_moving[3] = true;
    amax->pressed = true;
    ticks(2);
    CHECK(amax->triggered);
    CHECK(backs_off(amax));               // limit_enable is false here
    CHECK(!backs_off(zmax));              // never homed, never triggered

    // a motor alarm halts with the axis in the reason
    reset();
    auto *alarm = new endstop_info_t{true, 0, false, 'Y', 1, false};
    motor_alarms.push_back(alarm);
    ticks(1);
    CHECK(halted && halt_reason == MOTOR_ERROR_X + 1);

    // nothing runs once halted
    reset();
    halted = true;
    motor_moving[0] = true; xmax->pressed = true;
    ticks(50);
    CHECK(stopped.empty());

    printf(fails == 0 ? "endstops: all passed\n" : "endstops: %d FAILED\n", fails);
    return fails != 0;
}
