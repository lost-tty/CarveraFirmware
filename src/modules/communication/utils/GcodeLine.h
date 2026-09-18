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

class Words {
public:
    static const size_t INLINE = 8;
    Words() {}
    Words(const Words &o) { assign(o.begin(), o.size()); }
    Words(Words &&o) { take(o); }
    Words &operator=(const Words &o) { if (this != &o) assign(o.begin(), o.size()); return *this; }
    Words &operator=(Words &&o) { if (this != &o) { delete[] heap; take(o); } return *this; }
    ~Words() { delete[] heap; }

    void clear() { n = 0; }
    void reserve(size_t want);
    void push_back(const Word &w);
    size_t size() const { return n; }
    bool empty() const { return n == 0; }
    const Word &operator[](size_t i) const { return data()[i]; }
    Word &operator[](size_t i) { return data()[i]; }
    const Word *begin() const { return data(); }
    const Word *end() const { return data() + n; }
    Word *begin() { return data(); }
    Word *end() { return data() + n; }

private:
    const Word *data() const { return heap != nullptr ? heap : fixed; }
    Word *data() { return heap != nullptr ? heap : fixed; }
    void assign(const Word *from, size_t count);
    void take(Words &o);

    Word *heap = nullptr;
    size_t n = 0, capacity = INLINE;
    Word fixed[INLINE];
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
    const Words &words() const { return list; }

private:
    Words list;
    std::string err;
};

}
