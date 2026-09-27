#pragma once

#include "ConfigTable.h"
#include "SwitchConfig.h"

#define TEMP_CONTROL_CONFIG(X) \
    X(bool,  enable,          "enable",          true) \
    X(int,   get_m_code,      "get_m_code",      105) \
    X(str,   designator,      "designator",      "M", 4) \
    X(float, max_temp,        "max_temp",        60.0f) \
    X(float, min_temp,        "min_temp",        0.0f) \
    X(pin,   thermistor_pin,  "thermistor_pin",  "1.31") \
    X(float, beta,            "beta",            3950.0f) \
    X(float, r0,              "r0",              100000.0f) \
    X(float, t0,              "t0",              25.0f) \
    X(int,   r1,              "r1",              0) \
    X(int,   r2,              "r2",              4700)
CONFIG_STRUCT(SpindleTempConfigT, TEMP_CONTROL_CONFIG);


#define TEMP_SWITCH_CONFIG(X) \
    X(float, threshold_temp,        "threshold_temp",        35.0f) \
    X(float, cooldown_power_init,   "cooldown_power_init",   50.0f) \
    X(float, cooldown_power_step,   "cooldown_power_step",   10.0f) \
    X(float, cooldown_power_laser,  "cooldown_power_laser",  80.0f) \
    X(int,   cooldown_delay,        "cooldown_delay",        180) \
    X(enum,  fan_switch,            "switch",                "spindlefan", switch_names)
CONFIG_STRUCT(TempSwitchConfigT, TEMP_SWITCH_CONFIG);

// Only valid while the config is built: at boot and during a change notification.
extern const ConfigTable::Group temperature_control_pool_config_groups[];
inline const SpindleTempConfigT &spindle_temp_cfg()
{
    return ConfigTable::config<SpindleTempConfigT>(temperature_control_pool_config_groups);
}
inline const TempSwitchConfigT &temp_switch_cfg()
{
    return ConfigTable::config<TempSwitchConfigT>(&temperature_control_pool_config_groups[1]);
}
