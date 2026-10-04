#include "Parameters.h"
#include "Persist.h"

#include "libs/Kernel.h"
#include "GcodeDispatch.h"
#include "Robot.h"
#include "Conveyor.h"
#include "modules/robot/MachineTask.h"
#include "StepperMotor.h"
#include "checksumm.h"
#include "SpindlePublicAccess.h"
#include "SpindleControl.h"
#include "Player.h"
#include "Program.h"

#include <cstring>

#include <cmath>

static void machine_position(float *mpos)
{
    THEROBOT.get_real_machine_position(mpos);
}

static bool settle()
{
    if(machine_task.on_task()) return THECONVEYOR.wait_for_idle();
    return machine_task.post_drain();
}

static bool machine_state(int n)
{
    return n == 2000 || n == 3026 || n == 3027 || (n >= 5021 && n <= 5044);
}

bool Parameters::get(int n, float &v) const
{
    if (n >= 101 && n <= 120) {
        v = local[n - 101];
        return true;
    }
    if (n >= 501 && n <= 520) {
        v = persist.user_var(n - 501);
        return !std::isnan(v); // blank EEPROM reads as NaN
    }

    if (machine_state(n) && !settle()) return false;
    float mpos[3];
    switch (n) {
        case 2000: v = persist.tool_length(); return true;
        case 3026: v = persist.tool(); return true;
        case 3027: {
            struct spindle_status ss;
            v = 0;
            if(spindle_control != nullptr) {
                spindle_control->get_status(&ss);
                v = ss.current_rpm;
            }
            return true;
        }
        case 3033: v = player.m1_stops_program(); return true;
        case 5021: case 5022: case 5023:
            machine_position(mpos);
            v = mpos[n - 5021];
            return true;
        case 5041: case 5042: case 5043: {
            machine_position(mpos);
            Robot::wcs_t pos = THEROBOT.mcs2wcs(mpos);
            float w[3]{std::get<0>(pos), std::get<1>(pos), std::get<2>(pos)};
            v = THEROBOT.from_millimeters(w[n - 5041]);
            return true;
        }
#if MAX_ROBOT_ACTUATORS > 3
        case 5024: case 5044: v = THEROBOT.motor_position(A_AXIS); return true;
#endif
    }
    return false;
}

static bool spindle_on()
{
    struct spindle_status ss;
    if(spindle_control == nullptr) return false;
    spindle_control->get_status(&ss);
    return ss.state;
}

static bool player_playing()
{
    return program.playing();
}

Parameters::Table *Parameters::tables = nullptr;

void Parameters::add(Table &slot, const Named *rows, unsigned count, void *context)
{
    slot = Table{rows, count, context, tables};
    tables = &slot;
}

static float probe_axis(unsigned i)
{
    std::tuple<float, float, float, uint8_t> p = THEROBOT.get_last_probe_position();
    switch (i) {
        case 0: return std::get<0>(p);
        case 1: return std::get<1>(p);
        case 2: return std::get<2>(p);
        default: return std::get<3>(p);
    }
}

static float laser_mode(void *) { return (float)THEKERNEL->get_laser_mode(); }
static float homed(void *) { return (float)THEROBOT.is_homed_all_axes(); }
static float spindle_is_on(void *) { return (float)spindle_on(); }
static float playing(void *) { return (float)player_playing(); }
static float tlo(void *) { return persist.tool_length(); }
static float probe_x(void *) { return probe_axis(0); }
static float probe_y(void *) { return probe_axis(1); }
static float probe_z(void *) { return probe_axis(2); }
static float probe_ok(void *) { return probe_axis(3); }
static float cycle_initial(void *) { return gcode_dispatch.get_cycle_initial(); }

static constexpr Parameters::Named BUILTIN[] = {
    {"_laser_mode",  laser_mode,    true},
    {"_homed",       homed,         true},
    {"_spindle_on",  spindle_is_on, true},
    {"_playing",     playing,       false},
    {"_tlo",         tlo,           true},
    {"_probe_x",     probe_x,       true},
    {"_probe_y",     probe_y,       true},
    {"_probe_z",     probe_z,       true},
    {"_probe_ok",    probe_ok,      true},
    {"_cycle_initial", cycle_initial, false},
};

void Parameters::init()
{
    static Table slot;
    add(slot, BUILTIN, nullptr);
}

const Parameters::Named *Parameters::find(const char *name, void *&context)
{
    for (const Table *t = tables; t != nullptr; t = t->next) {
        for (unsigned i = 0; i < t->count; i++) {
            if (strcmp(t->rows[i].name, name) != 0) continue;
            context = t->context;
            return &t->rows[i];
        }
    }
    return nullptr;
}

bool Parameters::get_named(const char *name, float &v) const
{
    void *context;
    const Named *p = find(name, context);
    if (p == nullptr) return false;
    if (p->machine && !settle()) return false;
    v = p->get(context);
    return true;
}

bool Parameters::has_named(const char *name) const
{
    void *context;
    return find(name, context) != nullptr;
}

void Parameters::list_named(StreamOutput *stream)
{
    for (const Table *t = tables; t != nullptr; t = t->next) {
        for (unsigned i = 0; i < t->count; i++) {
            stream->printf("%-22s %.4f\n", t->rows[i].name, t->rows[i].get(t->context));
        }
    }
}

bool Parameters::set(int n, float v)
{
    if (n >= 101 && n <= 120) {
        local[n - 101] = v;
        return true;
    }
    if (n >= 501 && n <= 520) {
        persist.set_user_var(n - 501, v);
        return true;
    }
    return false;
}
