#pragma once

#include "ConfigTable.h"

// One struct serves the prefixes atc, atc.detector, atc.probe and coordinate.
#define ATC_ROOT_CONFIG(X) \
    X(pin,   homing_endstop_pin,   "homing_endstop_pin",     "1.0^") \
    X(int,   homing_debounce_ms,   "homing_debounce_ms",     1) \
    X(float, homing_max_travel_mm, "homing_max_travel_mm",   8.0f) \
    X(float, homing_retract_mm,    "homing_retract_mm",      0.4f) \
    X(float, homing_rate_mm_s,     "homing_rate_mm_s",       0.4f) \
    X(float, action_mm,            "action_mm",              1.6f) \
    X(float, action_rate_mm_s,     "action_rate_mm_s",       0.25f) \
    X(float, safe_z_mm,            "safe_z_mm",              -20.0f) \
    X(float, safe_z_empty_mm,      "safe_z_empty_mm",        -50.0f) \
    X(float, safe_z_offset_mm,     "safe_z_offset_mm",       15.0f) \
    X(float, fast_z_rate_mm_m,     "fast_z_rate_mm_m",       1000.0f) \
    X(float, slow_z_rate_mm_m,     "slow_z_rate_mm_m",       200.0f) \
    X(float, margin_rate_mm_m,     "margin_rate_mm_m",       1000.0f)

#define ATC_DETECTOR_CONFIG(X) \
    X(pin,   detect_pin,           "detect_pin",             "0.20^") \
    X(float, detect_rate_mm_s,     "detect_rate_mm_s",       20.0f) \
    X(float, detect_travel_mm,     "detect_travel_mm",       5.0f)

#define ATC_PROBE_CONFIG(X) \
    X(float, probe_fast_rate_mm_m, "fast_rate_mm_m",         500.0f) \
    X(float, probe_slow_rate_mm_m, "slow_rate_mm_m",         100.0f) \
    X(float, probe_retract_mm,     "retract_mm",             2.0f) \
    X(float, probe_height_mm,      "probe_height_mm",        0.0f)

#define ATC_COORDINATE_CONFIG(X) \
    X(float, anchor1_x,            "anchor1_x",              -360.158f) \
    X(float, anchor1_y,            "anchor1_y",              -234.568f) \
    X(float, anchor2_offset_x,     "anchor2_offset_x",       90.0f) \
    X(float, anchor2_offset_y,     "anchor2_offset_y",       45.0f) \
    X(float, toolrack_offset_x,    "toolrack_offset_x",      356.0f) \
    X(float, toolrack_offset_y,    "toolrack_offset_y",      0.0f) \
    X(float, toolrack_z,           "toolrack_z",             -112.5f) \
    X(float, rotation_offset_x,    "rotation_offset_x",      -8.0f) \
    X(float, rotation_offset_y,    "rotation_offset_y",      37.5f) \
    X(float, rotation_offset_z,    "rotation_offset_z",      22.35f) \
    X(float, clearance_x,          "clearance_x",            -75.0f) \
    X(float, clearance_y,          "clearance_y",            -3.0f) \
    X(float, clearance_z,          "clearance_z",            -3.0f)

#define ATC_ALL_CONFIG(X) \
    ATC_ROOT_CONFIG(X) ATC_DETECTOR_CONFIG(X) ATC_PROBE_CONFIG(X) ATC_COORDINATE_CONFIG(X)
CONFIG_STRUCT(ATCConfigT, ATC_ALL_CONFIG);

CONFIG_KEYS(atc_root_config_keys, ATCConfigT, ATC_ROOT_CONFIG);
CONFIG_KEYS(atc_detector_config_keys, ATCConfigT, ATC_DETECTOR_CONFIG);
CONFIG_KEYS(atc_probe_config_keys, ATCConfigT, ATC_PROBE_CONFIG);
CONFIG_KEYS(atc_coordinate_config_keys, ATCConfigT, ATC_COORDINATE_CONFIG);
