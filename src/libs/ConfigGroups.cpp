// The config groups of every module. A module with config keys must be listed here.
#include "ConfigTable.h"

extern const ConfigTable::Group
    kernel_config_groups[], main_config_groups[], conveyor_config_groups[],
    planner_config_groups[], robot_config_groups[], endstops_config_groups[],
    atc_config_groups[], laser_config_groups[], spindle_maker_config_groups[],
    switch_pool_config_groups[], temperature_control_pool_config_groups[],
    zprobe_config_groups[], cart_grid_strategy_config_groups[],
    three_point_strategy_config_groups[], main_button_config_groups[],
    wifi_provider_config_groups[], web_server_config_groups[],
    wireless_probe_config_groups[], usb_host_config_groups[];

const ConfigTable::Group *const config_groups[] = {
    kernel_config_groups, main_config_groups, conveyor_config_groups,
    planner_config_groups, robot_config_groups, endstops_config_groups,
    atc_config_groups, laser_config_groups, spindle_maker_config_groups,
    switch_pool_config_groups, temperature_control_pool_config_groups,
    zprobe_config_groups, cart_grid_strategy_config_groups,
    three_point_strategy_config_groups, main_button_config_groups,
    wifi_provider_config_groups, web_server_config_groups,
    wireless_probe_config_groups, usb_host_config_groups,
    nullptr,
};
