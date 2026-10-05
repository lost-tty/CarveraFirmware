#pragma once

class StreamOutput;

#include "GcodeLine.h"

// #101-#120 volatile, #501-#520 EEPROM, #2000/#3026/#3027/#3033/#5021-#5044 read-only machine state,
class Parameters : public gcode::ParamStore {
public:
    // Registers a module's constant parameter table. The caller owns the Table node.
    // machine: machine state, read only once the queue has run out
    typedef float (*getter)(void *context);
    struct Named { const char *name; getter get; bool machine; };
    struct Table { const Named *rows; unsigned count; void *context; Table *next; };
    template<unsigned N>
    static void add(Table &slot, const Named (&rows)[N], void *context)
    {
        add(slot, rows, N, context);
    }
    static void add(Table &slot, const Named *rows, unsigned count, void *context);

    bool get(int n, float &v) const override;
    bool set(int n, float v) override;
    bool get_named(const char *name, float &v) const override;
    bool has_named(const char *name) const override;
    bool set_named(const char *name, float v, std::string &err) override;  // only _motion_mode
    bool behind() const override { return behind_; }
    void clear_behind() { behind_ = false; }
    static void list_named(StreamOutput *stream);
    static void init();

private:
    static const Named *find(const char *name, void *&context);
    bool caught_up() const;
    static Table *tables;
    float local[20] = {};
    mutable bool behind_ = false;
};
