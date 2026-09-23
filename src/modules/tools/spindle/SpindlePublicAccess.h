#ifndef SPINDLEPUBLICACCESS_H
#define SPINDLEPUBLICACCESS_H

#include "checksumm.h"

#include <string>

struct spindle_status {
	bool state;
    float current_rpm;
    float target_rpm;
    float current_pwm_value;
	float factor;
};

#endif
