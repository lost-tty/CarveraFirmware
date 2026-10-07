#pragma once

#include "ConfigTable.h"
#include <cmath>

#define ROBOT_ROOT_CONFIG(X) \
    X(float, feed_rate,                 "default_feed_rate",         1000.0f) \
    X(float, seek_rate,                 "default_seek_rate",         3000.0f) \
    X(float, mm_per_line_segment,       "mm_per_line_segment",       5.0f) \
    X(float, delta_segments_per_second, "delta_segments_per_second", 0.0f) \
    X(float, mm_per_arc_segment,        "mm_per_arc_segment",        0.0f) \
    X(float, mm_max_arc_error,          "mm_max_arc_error",          0.002f) \
    X(int,   arc_correction,            "arc_correction",            5) \
    X(float, x_axis_max_speed,          "x_axis_max_speed",          4000.0f) \
    X(float, y_axis_max_speed,          "y_axis_max_speed",          4000.0f) \
    X(float, z_axis_max_speed,          "z_axis_max_speed",          4000.0f) \
    X(float, max_speed,                 "max_speed",                 -60.0f) \
    X(bool,  segment_z_moves,           "segment_z_moves",           true) \
    X(bool,  save_g92,                  "save_g92",                  false) \
    X(bool,  save_g54,                  "save_g54",                  true) \
    X(floats, set_g92,                  "set_g92",                   "", 3) \
    X(float, acceleration,              "acceleration",              150.0f) \
    X(float, z_acceleration,            "z_acceleration",            NAN) \
    X(bool,  home_on_boot,              "home_on_boot",              true) \
    X(float, laser_module_maximum_s_value, "laser_module_maximum_s_value", 1.0f) \
    X(float, laser_module_default_power,   "laser_module_default_power",   1.0f) \
    X(float, laser_module_offset_x,     "laser_module_offset_x",     -37.3f) \
    X(float, laser_module_offset_y,     "laser_module_offset_y",     4.8f) \
    X(float, laser_module_offset_z,     "laser_module_offset_z",     -45.0f)

CONFIG_STRUCT(RobotRootConfigT, ROBOT_ROOT_CONFIG);

#define ROBOT_SOFT_ENDSTOP_CONFIG(X) \
    X(bool,  enable, "enable", false) \
    X(bool,  halt,   "halt",   true) \
    X(float, x_min,  "x_min",  -371.0f) \
    X(float, y_min,  "y_min",  -250.0f) \
    X(float, z_min,  "z_min",  -135.0f)

CONFIG_STRUCT(RobotSoftEndstopConfigT, ROBOT_SOFT_ENDSTOP_CONFIG);

// A side left NAN is open.
#define ROBOT_KEEPOUT_CONFIG(X) \
    X(float, x_min, "x_min", NAN) \
    X(float, x_max, "x_max", NAN) \
    X(float, y_min, "y_min", NAN) \
    X(float, y_max, "y_max", NAN) \
    X(float, z_min, "z_min", NAN) \
    X(float, z_max, "z_max", NAN)

CONFIG_STRUCT(RobotKeepoutConfigT, ROBOT_KEEPOUT_CONFIG);

#define ROBOT_KEEPOUT_TOOLZ_CONFIG(X) \
    X(float, tool_z, "tool_z", -72.625f)

CONFIG_STRUCT(RobotKeepoutToolzConfigT, ROBOT_KEEPOUT_TOOLZ_CONFIG);

#define ROBOT_ACTUATOR_CONFIG(X) \
    X(pin,   step_pin,      "step_pin",      "nc") \
    X(pin,   dir_pin,       "dir_pin",       "nc") \
    X(pin,   en_pin,        "en_pin",        "nc") \
    X(float, steps_per_mm,  "steps_per_mm",  80.0f) \
    X(float, max_rate,      "max_rate",      30000.0f) \
    X(float, acceleration,  "acceleration",  NAN) \
    X(float, torque_knee,   "torque_knee",   NAN) \
    X(float, torque_end,    "torque_end",    NAN) \
    X(float, torque_floor,  "torque_floor",  NAN)

CONFIG_STRUCT(RobotActuatorConfigT, ROBOT_ACTUATOR_CONFIG);

CONFIG_KEYS(robot_root_config_keys, RobotRootConfigT, ROBOT_ROOT_CONFIG);
CONFIG_KEYS(robot_soft_endstop_config_keys, RobotSoftEndstopConfigT, ROBOT_SOFT_ENDSTOP_CONFIG);
CONFIG_KEYS(robot_keepout_config_keys, RobotKeepoutConfigT, ROBOT_KEEPOUT_CONFIG);
CONFIG_KEYS(robot_keepout_toolz_config_keys, RobotKeepoutToolzConfigT, ROBOT_KEEPOUT_TOOLZ_CONFIG);
CONFIG_KEYS(robot_actuator_config_keys, RobotActuatorConfigT, ROBOT_ACTUATOR_CONFIG);
