#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>

// Each module describes its config as a struct and a key table in flash. The structs exist
// only between build() and release(); modules copy the values they keep into their members.
class ConfigTable {
public:
    enum Type : uint8_t { FLOAT, INT, BOOL, STR, PIN, ENUM, GCODE, FLOATS };

    static const uint8_t ENUM_INVALID = 0xff;   // The text matched no entry of the key's list.
    static const uint16_t GCODE_NONE = 0xffff;  // M n is stored as n, G n as n | GCODE_G.
    static const size_t MAX_FIELD = 64;         // Largest STR or FLOATS field, in bytes.
    static const uint16_t GCODE_G = 0x4000;

    // A default or instance value. Bools are stored in i as 0 or 1.
    union Value {
        float f;
        int32_t i;
        const char *s;
        constexpr Value(float v) : f(v) {}
        constexpr Value(int32_t v) : i(v) {}
        constexpr Value(const char *v) : s(v) {}
    };

    // Selects the Value member from the field type, so a literal 0 for a float field is
    // stored as a float.
    template<class T> struct ValueOf;

    struct Key {
        const char *name;   // Suffix after the group prefix, e.g. "detector.detect_pin".
        uint16_t off;        // Offset into the group's struct.
        uint8_t type;
        uint8_t size;         // STR buffer size or FLOATS count.
        Value dflt;
        const char *const *names; // ENUM values, terminated by nullptr.
    };

    // A value that differs from the key table's default for one instance, such as
    // switch.vacuum or alpha_. It is matched to its key by offset.
    struct Override {
        uint16_t off;
        Value v;
    };

    struct Group;
    // Called after config set or reset with the rebuilt struct of the changed group.
    using Changed = void (*)(const Group *group, const void *cfg);

    // One group per config prefix: "" for root keys, "atc", "switch.vacuum" and so on. A
    // prefix ending in '_' joins its keys without a dot, as in alpha_steps_per_mm. A size of 0
    // means the keys live in the previous group's struct.
    struct Group {
        const char *prefix;
        const Key *keys;
        uint8_t n;
        uint16_t size;
        const Override *ov;
        uint8_t n_ov;
        Changed changed;
    };

    // Builds every struct from the defaults and path. Calls nest, so a build/release pair may
    // run while the boot build is up. Returns false when out of memory.
    static bool build(const char *path);
    static void release();

    // Only valid between build() and release().
    static const void *block(const Group *group);
    template<class T> static const T &config(const Group *group) {
        return *(const T *)block(group);
    }

    // Writes the key's line into path and notifies the owning module. Any key may be stored.
    static bool set(const char *path, const char *name, const char *value);

    // Removes the key's lines from path and notifies the owning module.
    static bool unset(const char *path, const char *name);

    // Returns false if no registered group has the key.
    static bool find(const char *name, const Group **group, const Key **key);

    // The value in effect (config.txt or default), spelled as in config.txt. Returns false
    // if no group has the key.
    static bool get(const char *name, char *buf, size_t bufsize);

    // Formats a field of the built structs, or its compiled-in default, as config.txt spells it.
    static void format(const Group *group, const Key *key, char *buf, size_t bufsize);
    static void format_default(const Group *group, const Key *key, char *buf, size_t bufsize);

    using EachFn = void (*)(const char *name, const Group *group, const Key *key, void *user);
    static void for_each(EachFn fn, void *user);

    // A missing file is not an error.
    static void load(const char *path);

    // Sets setting's line to "setting value", keeping a trailing comment and dropping later
    // duplicates. A missing key is appended. Returns false on I/O failure.
    static bool rewrite(const char *path, const char *setting, const char *value);

    // Removes every line of setting from path. Returns false on I/O failure.
    static bool remove_key(const char *path, const char *setting);

private:
    static char *base(const Group *group);
    static void parse_line(const char *line);
    static void set_field(void *base, const Key *key, const char *text);
    static void set_value(void *base, const Key *key, Value v);
    static void notify(const char *name);
};

template<> struct ConfigTable::ValueOf<float> {
    static constexpr Value make(float v) { return Value(v); }
};
template<> struct ConfigTable::ValueOf<int32_t> {
    static constexpr Value make(int32_t v) { return Value(v); }
};
template<> struct ConfigTable::ValueOf<bool> {
    static constexpr Value make(bool v) { return Value((int32_t)(v ? 1 : 0)); }
};
// Pins, enums, gcodes and float lists take text defaults, parsed by build().
template<> struct ConfigTable::ValueOf<uint16_t> {
    static constexpr Value make(const char *v) { return Value(v); }
};
template<> struct ConfigTable::ValueOf<uint8_t> {
    static constexpr Value make(const char *v) { return Value(v); }
};
template<size_t N> struct ConfigTable::ValueOf<float[N]> {
    static constexpr Value make(const char *v) { return Value(v); }
};
template<size_t N> struct ConfigTable::ValueOf<char[N]> {
    static constexpr Value make(const char *v) { return Value(v); }
};

// Every module's groups, one null-terminated array each. Defined in ConfigGroups.cpp.
extern const ConfigTable::Group *const config_groups[];

// X(type, field, "name", default)
#define CFG_FLOAT(field, name, dflt) float field;
#define CFG_INT(field, name, dflt) int32_t field;
#define CFG_BOOL(field, name, dflt) bool field;
#define CFG_STR(field, name, dflt, size) \
    char field[size]; static_assert(size <= ConfigTable::MAX_FIELD, "config string too long");
#define CFG_PIN(field, name, dflt) uint16_t field;   /* a PinSpec */
#define CFG_ENUM(field, name, dflt, names) uint8_t field;
#define CFG_GCODE(field, name, dflt) uint16_t field;
#define CFG_FLOATS(field, name, dflt, n) \
    float field[n]; \
    static_assert(n * sizeof(float) <= ConfigTable::MAX_FIELD, "config list too long");

#define CONFIG_STRUCT(Struct, LIST) \
    struct Struct { LIST(CFG_FIELD) }

#define CFG_FIELD(kind, ...) CFG_FIELD_##kind(__VA_ARGS__)
#define CFG_FIELD_float(field, name, dflt) CFG_FLOAT(field, name, dflt)
#define CFG_FIELD_int(field, name, dflt) CFG_INT(field, name, dflt)
#define CFG_FIELD_bool(field, name, dflt) CFG_BOOL(field, name, dflt)
#define CFG_FIELD_str(field, name, dflt, size) CFG_STR(field, name, dflt, size)
#define CFG_FIELD_pin(field, name, dflt) CFG_PIN(field, name, dflt)
#define CFG_FIELD_enum(field, name, dflt, names) CFG_ENUM(field, name, dflt, names)
#define CFG_FIELD_gcode(field, name, dflt) CFG_GCODE(field, name, dflt)
#define CFG_FIELD_floats(field, name, dflt, n) CFG_FLOATS(field, name, dflt, n)

#define CFG_KEYROW(kind, ...) CFG_KEYROW_##kind(__VA_ARGS__)
#define CFG_DFLT(field, dflt) ConfigTable::ValueOf<decltype(STRUCT_T::field)>::make(dflt)
#define CFG_KEYROW_float(field, name, dflt) \
    { name, offsetof(STRUCT_T, field), ConfigTable::FLOAT, 0, CFG_DFLT(field, dflt), nullptr },
#define CFG_KEYROW_int(field, name, dflt) \
    { name, offsetof(STRUCT_T, field), ConfigTable::INT, 0, CFG_DFLT(field, dflt), nullptr },
#define CFG_KEYROW_bool(field, name, dflt) \
    { name, offsetof(STRUCT_T, field), ConfigTable::BOOL, 0, CFG_DFLT(field, dflt), nullptr },
#define CFG_KEYROW_str(field, name, dflt, size) \
    { name, offsetof(STRUCT_T, field), ConfigTable::STR, size, CFG_DFLT(field, dflt), nullptr },
#define CFG_KEYROW_pin(field, name, dflt) \
    { name, offsetof(STRUCT_T, field), ConfigTable::PIN, 0, CFG_DFLT(field, dflt), nullptr },
#define CFG_KEYROW_enum(field, name, dflt, names) \
    { name, offsetof(STRUCT_T, field), ConfigTable::ENUM, 0, ConfigTable::Value(dflt), names },
#define CFG_KEYROW_gcode(field, name, dflt) \
    { name, offsetof(STRUCT_T, field), ConfigTable::GCODE, 0, ConfigTable::Value(dflt), nullptr },
#define CFG_KEYROW_floats(field, name, dflt, n) \
    { name, offsetof(STRUCT_T, field), ConfigTable::FLOATS, n, ConfigTable::Value(dflt), nullptr },

// offsetof() needs the struct by a fixed name, STRUCT_T, so each table defines it in a
// namespace of its own to avoid collisions within one translation unit.
#define CONFIG_KEYS(table_name, Struct, LIST) \
    namespace table_name##_ns { typedef Struct STRUCT_T; \
        static const ConfigTable::Key rows[] = { LIST(CFG_KEYROW) }; } \
    enum { table_name##_count = \
        sizeof(table_name##_ns::rows) / sizeof(ConfigTable::Key) }

// An instance value, e.g. CFG_SET(SwitchConfigT, output_pin, "2.13").
#define CFG_SET(Struct, field, v) \
    { offsetof(Struct, field), ConfigTable::ValueOf<decltype(Struct::field)>::make(v) }

// A CONFIG_GROUPS() entry. keys names a CONFIG_KEYS table; changed may be nullptr.
#define CFG_GROUP(prefix, keys, Struct, changed) \
    { prefix, keys##_ns::rows, keys##_count, sizeof(Struct), nullptr, 0, changed }
#define CFG_GROUP_OV(prefix, keys, Struct, ov, changed) \
    { prefix, keys##_ns::rows, keys##_count, sizeof(Struct), ov, sizeof(ov) / sizeof((ov)[0]), \
      changed }
// More keys of the previous group's struct.
#define CFG_GROUP_MORE(prefix, keys, changed) \
    { prefix, keys##_ns::rows, keys##_count, 0, nullptr, 0, changed }

// A module's groups. List the name in ConfigGroups.cpp.
#define CONFIG_GROUPS(name, ...) \
    extern const ConfigTable::Group name[]; \
    const ConfigTable::Group name[] = { \
        __VA_ARGS__, { nullptr, nullptr, 0, 0, nullptr, 0, nullptr } }
