#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gcode {

struct Word {
    char letter;
    uint8_t subcode; // G and M only
    float value;
    bool has_value; // "G28 X Y": a bare letter reads as 0
};

class ParamStore {
public:
    virtual ~ParamStore() {}
    virtual bool get(int n, float &v) const = 0;
    virtual bool set(int n, float v) = 0;
    virtual bool get_named(const char *name, float &v) const { (void)name; (void)v; return false; } // #<name>, scripts only
    virtual bool set_named(const char *name, float v, std::string &err) { (void)name; (void)v; (void)err; return false; }
    bool exists_named(const char *name) const { float v; return get_named(name, v); }
};

inline void skip_space(const char *&p) { while (*p == ' ' || *p == '\t') p++; }

// Expression: number, #n, #<name>, [expr], FUNC[expr]; ** * / MOD + - EQ NE GT GE LT LE AND OR XOR
// with LinuxCNC precedence and unary sign. Advances p.
bool eval(const char *&p, float &out, const ParamStore *params, std::string &err);

// reads a #<name> at p (p on '<'), returns false if malformed
bool named_param(const char *&p, std::string &name, std::string &err);

// "#n = expr", "#<name> = expr" or "#[expr] = expr" with p on the '#'; a trailing comment is allowed
bool assign(const char *p, ParamStore &store, std::string &err);

// Operators need brackets: X[#101+1]. Only G and M may repeat.
class Line {
public:
    bool parse(const char *text, const ParamStore *params);
    const std::string &error_text() const { return err; }
    const std::vector<Word> &words() const { return list; }

private:
    std::vector<Word> list;
    std::string err;
};

}
