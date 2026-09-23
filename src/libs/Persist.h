#pragma once

#include <stdint.h>

namespace mbed { class I2C; }

// What survives a power cycle. The layout is on the chip, so fields are only ever appended.
class Persist {
public:
    void init(mbed::I2C *bus);

    int   tool() const { return record.tool; }
    float tool_length() const { return record.tool_length; }
    float tool_z() const { return record.tool_z; }
    float reference_z() const { return record.reference_z; }
    float work_offset(uint8_t wcs, uint8_t axis) const;
    float user_var(uint8_t n) const { return record.user_vars[n]; }

    void set_tool(int t) { record.tool = t; save(); }
    void set_tool_length(float mm) { record.tool_length = mm; save(); }
    void set_tool_z(float mz) { record.tool_z = mz; save(); }
    void set_reference_z(float mz) { record.reference_z = mz; save(); }
    void set_work_offset(uint8_t wcs, float x, float y, float z);
    void set_user_var(uint8_t n, float v) { record.user_vars[n] = v; save(); }

    static const uint8_t k_user_vars = 20;   // #501 to #520
    static const uint8_t k_work_offsets = 9; // G54 to G59.3
    bool erase();

private:
    struct Record {
        float tool_length;
        float work_offset[3];
        float reference_z;
        float tool_z;
        float unused;
        int   tool;
        float user_vars[k_user_vars];
        float more_work_offsets[k_work_offsets - 1][3];
    };

    void save();
    bool store(const void *from);
    bool page_write(uint8_t page, uint8_t len, const uint8_t *from);

    mbed::I2C *i2c{nullptr};
    Record record{};
    Record written{};
};

extern Persist persist;
