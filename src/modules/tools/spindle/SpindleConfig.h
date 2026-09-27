#pragma once

#include "ConfigTable.h"

static const char *const spindle_types[] = { "pwm", "analog", "modbus", nullptr };
enum : uint8_t { SPINDLE_PWM, SPINDLE_ANALOG, SPINDLE_MODBUS };
static const char *const spindle_vfd_types[] = { "none", "huanyang", nullptr };
enum : uint8_t { VFD_NONE, VFD_HUANYANG };

// Shared by all spindle types; each reads only the keys it uses.
#define SPINDLE_CONFIG(X) \
    X(bool,  enable,             "enable",             true) \
    X(enum,  type,               "type",               "pwm", spindle_types) \
    X(enum,  vfd_type,           "vfd_type",           "none", spindle_vfd_types) \
    X(pin,   pwm_pin,            "pwm_pin",            "2.5") \
    X(int,   pwm_period,         "pwm_period",         1000) \
    X(float, max_pwm,            "max_pwm",            1.0f) \
    X(pin,   feedback_pin,       "feedback_pin",       "2.7") \
    X(float, pulses_per_rev,     "pulses_per_rev",     12.0f) \
    X(float, default_rpm,        "default_rpm",        10000.0f) \
    X(float, control_p,          "control_P",          0.00001f) \
    X(float, control_i,          "control_I",          0.00005f) \
    X(float, control_d,          "control_D",          0.00005f) \
    X(float, control_smoothing,  "control_smoothing",  0.1f) \
    X(float, delay_s,            "delay_s",            3.0f) \
    X(float, acc_ratio,          "acc_ratio",          1.635f) \
    X(pin,   alarm_pin,          "alarm_pin",          "0.19^") \
    X(int,   stall_s,            "stall_s",            100) \
    X(int,   stall_count_rpm,    "stall_count_rpm",    8000) \
    X(int,   stall_alarm_rpm,    "stall_alarm_rpm",    5000) \
    X(int,   min_rpm,            "min_rpm",            100) \
    X(int,   max_rpm,            "max_rpm",            5000) \
    X(pin,   switch_on_pin,      "switch_on_pin",      "nc") \
    X(pin,   rx_pin,             "rx_pin",             "nc") \
    X(pin,   tx_pin,             "tx_pin",             "nc") \
    X(pin,   dir_pin,            "dir_pin",            "nc")
CONFIG_STRUCT(SpindleConfigT, SPINDLE_CONFIG);

extern const ConfigTable::Group spindle_maker_config_groups[];
// Only valid while the config is built: at boot and during a change notification.
inline const SpindleConfigT &spindle_cfg()
{
    return ConfigTable::config<SpindleConfigT>(spindle_maker_config_groups);
}
