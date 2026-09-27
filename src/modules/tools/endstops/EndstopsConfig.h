#pragma once

#include "ConfigTable.h"

static const char *const homing_directions[] = { "home_to_min", "home_to_max", nullptr };

#define ENDSTOP_AXIS_CONFIG(X) \
    X(pin,   min_endstop,        "min_endstop",           "nc") \
    X(pin,   max_endstop,        "max_endstop",           "nc") \
    X(float, max_travel,         "max_travel",            500.0f) \
    X(float, fast_homing_rate,   "fast_homing_rate_mm_s", 100.0f) \
    X(float, slow_homing_rate,   "slow_homing_rate_mm_s", 3.0f) \
    X(float, homing_retract_mm,  "homing_retract_mm",     1.0f) \
    /* anything but "home_to_max" homes to min */ \
    X(enum,  homing_direction,   "homing_direction",      "home_to_max", homing_directions) \
    X(float, min_pos,            "min",                   0.0f) \
    X(float, max_pos,            "max",                   0.0f) \
    X(bool,  limit_enable,       "limit_enable",          true) \
    X(pin,   motor_alarm_pin,    "motor_alarm_pin",       "nc")

CONFIG_STRUCT(EndstopAxisConfigT, ENDSTOP_AXIS_CONFIG);
CONFIG_KEYS(endstop_axis_config_keys, EndstopAxisConfigT, ENDSTOP_AXIS_CONFIG);

#define ENDSTOPS_GLOBAL_CONFIG(X) \
    X(bool,  module_enable,  "endstops_enable",       true) \
    X(float, hysteresis_mm,  "endstop_hysteresis_mm", 0.1f) \
    X(bool,  home_z_first,   "home_z_first",          true) \
    X(pin,   cover_endstop,  "cover_endstop",         "1.9^") \
    X(str,   homing_order,   "homing_order",          "", 8)

CONFIG_STRUCT(EndstopsGlobalConfigT, ENDSTOPS_GLOBAL_CONFIG);
CONFIG_KEYS(endstops_global_config_keys, EndstopsGlobalConfigT, ENDSTOPS_GLOBAL_CONFIG);
