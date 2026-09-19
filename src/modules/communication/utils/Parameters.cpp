#include "Parameters.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "checksumm.h"
#include "SpindlePublicAccess.h"
#include "SpindleControl.h"
#include "ATCHandlerPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "Player.h"

#include <cstring>

#include <cmath>

static void machine_position(float *mpos)
{
    THEROBOT.get_current_machine_position(mpos);
    // machine_position includes the compensation transform, undo it to report the real position
    if (THEROBOT.compensationTransform) THEROBOT.compensationTransform(mpos, true, false);
}

bool Parameters::get(int n, float &v) const
{
    if (n >= 101 && n <= 120) {
        v = local[n - 101];
        return true;
    }
    if (n >= 501 && n <= 520) {
        v = THEKERNEL->eeprom_data.perm_vars[n - 501];
        return !std::isnan(v); // blank EEPROM reads as NaN
    }

    if (n >= 5021 && n <= 5044) THECONVEYOR.wait_for_idle(); // positions are where the machine is, not where it is going
    float mpos[3];
    switch (n) {
        case 2000: v = THEKERNEL->eeprom_data.TLO; return true;
        case 3026: v = THEKERNEL->eeprom_data.TOOL; return true;
        case 3027: {
            struct spindle_status ss;
            v = 0;
            if(spindle_control != nullptr) {
                spindle_control->get_status(&ss);
                v = ss.current_rpm;
            }
            return true;
        }
        case 3033: v = THEKERNEL->get_optional_stop_mode(); return true;
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

Parameters::Named *Parameters::named = nullptr;

void Parameters::add(Named &slot, const char *name, getter get, void *context)
{
    slot = Named{name, get, context, named};
    named = &slot;
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

static const struct { const char *name; Parameters::getter get; } BUILTIN[] = {
    {"_laser_mode",  [](void *) { return (float)THEKERNEL->get_laser_mode(); }},
    {"_homed",       [](void *) { return (float)THEROBOT.is_homed_all_axes(); }},
    {"_spindle_on",  [](void *) { return (float)spindle_on(); }},
    {"_playing",     [](void *) { return (float)player_playing(); }},
    {"_tlo",         [](void *) { return THEKERNEL->eeprom_data.TLO; }},
    {"_probe_x",     [](void *) { return probe_axis(0); }},
    {"_probe_y",     [](void *) { return probe_axis(1); }},
    {"_probe_z",     [](void *) { return probe_axis(2); }},
    {"_probe_ok",    [](void *) { return probe_axis(3); }},
};

void Parameters::init()
{
    static Named slots[sizeof(BUILTIN) / sizeof(*BUILTIN)];
    for (unsigned i = 0; i < sizeof(BUILTIN) / sizeof(*BUILTIN); i++) {
        add(slots[i], BUILTIN[i].name, BUILTIN[i].get, nullptr);
    }
}

bool Parameters::get_named(const char *name, float &v) const
{
    for (const Named *p = named; p != nullptr; p = p->next) {
        if (strcmp(p->name, name) == 0) {
            v = p->get(p->context);
            return true;
        }
    }
    return false;
}

void Parameters::list_named(StreamOutput *stream)
{
    for (const Named *p = named; p != nullptr; p = p->next) stream->printf("%-22s %.4f\n", p->name, p->get(p->context));
}

bool Parameters::set(int n, float v)
{
    if (n >= 101 && n <= 120) {
        local[n - 101] = v;
        return true;
    }
    if (n >= 501 && n <= 520) {
        if (THEKERNEL->eeprom_data.perm_vars[n - 501] == v) return true; // the write blocks the main loop for ~0.4 s
        THEKERNEL->eeprom_data.perm_vars[n - 501] = v;
        THEKERNEL->write_eeprom_data();
        return true;
    }
    return false;
}
