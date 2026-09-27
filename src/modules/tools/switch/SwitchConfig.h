#pragma once

#include "ConfigTable.h"

// The machine's switches, in the order SwitchPool creates them.
static const char *const switch_names[] = {
    "vacuum", "air", "light", "spindlefan", "extend", "toolsensor", "probecharger", nullptr };

static const char *const switch_output_types[] = {
    "pwm", "digital", "hwpwm", "swpwm", "digitalpwm", nullptr };
enum : uint8_t { SW_OUT_PWM, SW_OUT_DIGITAL, SW_OUT_HWPWM, SW_OUT_SWPWM, SW_OUT_DIGITALPWM };
enum : uint8_t { SW_IN_MOMENTARY, SW_IN_TOGGLE };

#define SWITCH_CONFIG(X) \
    X(bool, enable,               "enable",               true) \
    X(int,  subcode,              "subcode",              0) \
    X(gcode, input_on_command,    "input_on_command",     "") \
    X(gcode, input_off_command,   "input_off_command",    "") \
    X(bool, startup_state,        "startup_state",        false) \
    X(enum, output_type,          "output_type",          "digital", switch_output_types) \
    X(int,  failsafe_set_to,      "failsafe_set_to",      0) \
    X(bool, ignore_on_halt,       "ignore_on_halt",        false) \
    X(pin,  output_pin,           "output_pin",           "nc") \
    X(pin,  pwm_pin,              "pwm_pin",              "nc") \
    /* NAN: the output type's own default applies (Switch::on_config_reload) */ \
    X(float, max_pwm,             "max_pwm",              NAN) \
    X(float, min_pwm,             "min_pwm",              NAN) \
    X(float, startup_value,       "startup_value",        NAN) \
    X(float, default_on_value,    "default_on_value",     NAN) \
    X(float, pwm_period_ms,       "pwm_period_ms",        20.0f)
CONFIG_STRUCT(SwitchConfigT, SWITCH_CONFIG);
