#pragma once

#include "checksumm.h"

#define scripts_checksum    CHECKSUM("scripts")
#define run_script_checksum CHECKSUM("run_script")

// PublicData::set_value(scripts_checksum, run_script_checksum, &call) starts the sub when the machine
// script defines it and nothing else is running (taken); otherwise nothing happens
struct script_call {
    const char *sub;
    const float *args;
    unsigned nargs;
};
