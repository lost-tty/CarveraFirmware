#pragma once

#include "GcodeLine.h"

// #101-#120 volatile, #501-#520 EEPROM, #2000/#3026/#3027/#3033/#5021-#5044 read-only machine state
class Parameters : public gcode::ParamStore {
public:
    bool get(int n, float &v) const override;
    bool set(int n, float v) override;

private:
    float local[20] = {};
};
