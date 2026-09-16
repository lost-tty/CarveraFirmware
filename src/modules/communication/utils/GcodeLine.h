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
};

// Expression: number, #param, [expr]; operators + - * / with unary sign. Advances p.
bool eval(const char *&p, float &out, const ParamStore *params, std::string &err);

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
