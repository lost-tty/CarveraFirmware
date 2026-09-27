#pragma once

#include "ConfigTable.h"

#define ZPROBE_CONFIG(X) \
    X(bool, enable,               "enable",               true) \
    X(pin,  probe_pin,            "probe_pin",            "2.6v") \
    X(pin,  calibrate_pin,        "calibrate_pin",        "0.5^") \
    X(float, probe_height,        "probe_height",         2.0f) \
    X(float, slow_feedrate,       "slow_feedrate",        1.5f) \
    X(float, fast_feedrate,       "fast_feedrate",        5.0f) \
    X(float, return_feedrate,     "return_feedrate",      20.0f) \
    X(bool, reverse_z,            "reverse_z",            false) \
    X(float, max_z,               "max_z",                100.0f) \
    X(float, dwell_before_probing,"dwell_before_probing", 0.0f)
CONFIG_STRUCT(ZProbeConfigT, ZPROBE_CONFIG);

#define THREE_POINT_CONFIG(X) \
    X(bool, enable,          "enable",          false) \
    X(floats, point1,        "point1",          "", 2) \
    X(floats, point2,        "point2",          "", 2) \
    X(floats, point3,        "point3",          "", 2) \
    X(floats, probe_offsets, "probe_offsets",   "0,0,0", 3) \
    X(bool, home_first,      "home_first",      true) \
    X(float, tolerance,      "tolerance",       0.03f) \
    X(bool, save_plane,      "save_plane",      false)
CONFIG_STRUCT(ThreePointConfigT, THREE_POINT_CONFIG);
extern const ConfigTable::Group three_point_strategy_config_groups[];
inline const ThreePointConfigT &threepoint_cfg()
{
    return ConfigTable::config<ThreePointConfigT>(three_point_strategy_config_groups);
}

// A grid_x_size or grid_y_size of -1 falls back to size.
#define CART_GRID_CONFIG(X) \
    X(bool, enable,               "enable",               true) \
    X(int,  grid_size,            "size",                 15) \
    X(int,  grid_x_size,          "grid_x_size",          -1) \
    X(int,  grid_y_size,          "grid_y_size",          -1) \
    X(float, tolerance,           "tolerance",            0.03f) \
    X(bool, save,                 "save",                 false) \
    X(bool, do_home,              "do_home",              true) \
    X(bool, only_by_two_corners,  "only_by_two_corners",  true) \
    X(bool, human_readable,       "human_readable",       true) \
    X(bool, do_manual_attach,     "m_attach",             false) \
    X(float, height_limit,        "height_limit",         NAN) \
    X(float, dampening_start,     "dampening_start",      NAN) \
    X(float, x_size,              "x_size",               50.0f) \
    X(float, y_size,              "y_size",               50.0f) \
    X(float, initial_height,      "initial_height",       NAN) \
    X(floats, probe_offsets,      "probe_offsets",        "0,0,0", 3) \
    X(floats, mount_position,     "mount_position",       "0,0,50", 3)
CONFIG_STRUCT(CartGridConfigT, CART_GRID_CONFIG);
extern const ConfigTable::Group cart_grid_strategy_config_groups[];
inline const CartGridConfigT &cartgrid_cfg()
{
    return ConfigTable::config<CartGridConfigT>(cart_grid_strategy_config_groups);
}
