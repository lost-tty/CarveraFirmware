#include "Parameters.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "PublicData.h"
#include "SpindlePublicAccess.h"

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

    float mpos[3];
    switch (n) {
        case 2000: v = THEKERNEL->eeprom_data.TLO; return true;
        case 3026: v = THEKERNEL->eeprom_data.TOOL; return true;
        case 3027: {
            struct spindle_status ss;
            v = PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss) ? ss.current_rpm : 0;
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
        case 5024: case 5044: v = THEROBOT.actuators[A_AXIS]->get_current_position(); return true;
#endif
    }
    return false;
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
