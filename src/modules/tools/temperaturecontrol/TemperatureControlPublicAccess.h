#ifndef __TEMPERATURECONTROLPUBLICACCESS_H
#define __TEMPERATURECONTROLPUBLICACCESS_H

#include "checksumm.h"

#include <string>

#define temperature_control_checksum      CHECKSUM("temperature_control")
#define spindle_temperature_checksum	  CHECKSUM("spindle")

struct pad_temperature {
    float current_temperature;
    float target_temperature;
    int pwm;
    uint16_t id;
    std::string designator;
};
#endif
