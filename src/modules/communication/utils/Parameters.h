#pragma once

class StreamOutput;

#include "GcodeLine.h"

// #101-#120 volatile, #501-#520 EEPROM, #2000/#3026/#3027/#3033/#5021-#5044 read-only machine state,
// #<_name> read-only machine state and ATC configuration for scripts
class Parameters : public gcode::ParamStore {
public:
    // a module publishes its #<_name> values here; the slot lives in the caller, as with shell commands
    typedef float (*getter)(void *context);
    struct Named { const char *name; getter get; void *context; Named *next; };
    static void add(Named &slot, const char *name, getter get, void *context);

    bool get(int n, float &v) const override;
    bool set(int n, float v) override;
    bool get_named(const char *name, float &v) const override;
    static void list_named(StreamOutput *stream);
    static void init();

private:
    static Named *named;
    float local[20] = {};
};
