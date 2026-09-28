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

#include <cstring>

#include <cmath>

static void machine_position(float *mpos)
{
    THEROBOT.get_real_machine_position(mpos);
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

    // the queue has to run out first, or this reads the planned position
    if (n >= 5021 && n <= 5044) machine_task.post_drain();
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
    return player.is_playing();
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
    {"_laser_mode",  laser_mode},
    {"_homed",       homed},
    {"_spindle_on",  spindle_is_on},
    {"_playing",     playing},
    {"_tlo",         tlo},
    {"_probe_x",     probe_x},
    {"_probe_y",     probe_y},
    {"_probe_z",     probe_z},
    {"_probe_ok",    probe_ok},
    {"_cycle_initial", cycle_initial},
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
    v = p->get(context);
    return true;
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
