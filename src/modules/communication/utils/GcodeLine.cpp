#include "GcodeLine.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace gcode {

static void skip_space(const char *&p)
{
    while (*p == ' ' || *p == '\t') p++;
}

static bool is_digit(char c)
{
    return c >= '0' && c <= '9';
}

static bool starts_value(char c)
{
    return is_digit(c) || c == '.' || c == '#' || c == '[' || c == '-' || c == '+';
}

static bool number(const char *&p, float &out, std::string &err)
{
    const char *s = p;
    while (is_digit(*p)) p++;
    if (*p == '.') {
        p++;
        while (is_digit(*p)) p++;
    }
    size_t n = p - s;
    bool exponent = *p == 'E' || *p == 'e'; // no exponents in RS274, and X1E5 is not an E word either
    if (n == 0 || (n == 1 && *s == '.') || n > 31 || exponent) {
        err = "bad number";
        return false;
    }
    char buf[32];
    memcpy(buf, s, n);
    buf[n] = 0;
    out = strtof(buf, nullptr);
    return true;
}

static const int MAX_DEPTH = 4; // brackets and signs nest on the main-loop stack, ~190 bytes each

static bool unary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth);
static bool expr(const char *&p, float &out, const ParamStore *params, std::string &err, int depth);

static bool primary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    if (depth > MAX_DEPTH) {
        err = "expression too deep";
        return false;
    }
    skip_space(p);
    if (*p == '[') {
        p++;
        if (!expr(p, out, params, err, depth + 1)) return false;
        skip_space(p);
        if (*p != ']') {
            err = "missing ]";
            return false;
        }
        p++;
        return true;
    }
    if (*p == '#') {
        p++;
        float n;
        if (!unary(p, n, params, err, depth + 1)) return false;
        if (!(n >= 0 && n <= 99999) || n != (int)n) {
            err = "bad parameter number";
            return false;
        }
        if (params == nullptr || !params->get((int)n, out)) {
            char buf[32];
            snprintf(buf, sizeof(buf), "no value for parameter #%d", (int)n);
            err = buf;
            return false;
        }
        return true;
    }
    return number(p, out, err);
}

static bool unary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    bool neg = false;
    for (skip_space(p); *p == '-' || *p == '+'; skip_space(p)) neg ^= *p++ == '-';
    if (!primary(p, out, params, err, depth)) return false;
    if (neg) out = -out;
    return true;
}

static bool term(const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    if (!unary(p, out, params, err, depth)) return false;
    for (;;) {
        skip_space(p);
        char op = *p;
        if (op != '*' && op != '/') return true;
        p++;
        float rhs;
        if (!unary(p, rhs, params, err, depth)) return false;
        if (op == '/' && rhs == 0) {
            err = "division by zero";
            return false;
        }
        out = op == '*' ? out * rhs : out / rhs;
    }
}

static bool expr(const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    if (!term(p, out, params, err, depth)) return false;
    for (;;) {
        skip_space(p);
        char op = *p;
        if (op != '+' && op != '-') return true;
        p++;
        float rhs;
        if (!term(p, rhs, params, err, depth)) return false;
        out = op == '+' ? out + rhs : out - rhs;
    }
}

bool eval(const char *&p, float &out, const ParamStore *params, std::string &err)
{
    if (!expr(p, out, params, err, 0)) return false;
    if (!std::isfinite(out)) {
        err = "value out of range";
        return false;
    }
    return true;
}

static bool code(const char *&p, Word &w, std::string &err)
{
    skip_space(p);
    if (!is_digit(*p)) {
        err = "bad code";
        return false;
    }
    long n = strtol(p, const_cast<char **>(&p), 10);
    if (n > 9999) {
        err = "bad code";
        return false;
    }
    w.value = n;
    w.has_value = true;
    if (*p != '.') return true;
    p++;
    if (!is_digit(*p)) {
        err = "bad subcode";
        return false;
    }
    n = strtol(p, const_cast<char **>(&p), 10);
    if (n > 99) {
        err = "bad subcode";
        return false;
    }
    w.subcode = n;
    return true;
}

bool Line::parse(const char *p, const ParamStore *params)
{
    list.clear();
    list.reserve(8);
    err.clear();
    for (;;) {
        skip_space(p);
        char c = *p;
        if (c == 0 || c == ';' || c == '\r' || c == '\n') return true;
        if (c == '(') {
            while (*p && *p != ')') p++;
            if (*p == 0) return true; // unterminated comment runs to end of line like Grbl
            p++;
            continue;
        }
        if (c == '%') {
            p++;
            continue;
        }
        char letter = toupper((unsigned char)c);
        if (letter < 'A' || letter > 'Z') {
            char buf[24];
            if (c >= ' ' && c < 127) snprintf(buf, sizeof(buf), "unexpected '%c'", c);
            else snprintf(buf, sizeof(buf), "unexpected byte 0x%02X", (unsigned char)c);
            err = buf;
            return false;
        }
        p++;
        Word w{letter, 0, 0, false};
        if (letter == 'G' || letter == 'M') {
            if (!code(p, w, err)) return false;
        } else {
            skip_space(p);
            if (starts_value(*p)) {
                if (!unary(p, w.value, params, err, 0)) return false;
                w.has_value = true;
                if (!std::isfinite(w.value)) {
                    err = "value out of range";
                    return false;
                }
                skip_space(p);
                if (*p == '+' || *p == '-' || *p == '*' || *p == '/') {
                    err = "operators need brackets";
                    return false;
                }
            }
            if (letter == 'N') continue;
            for (const Word &o : list) {
                if (o.letter == letter) {
                    err = std::string("duplicate word ") + letter;
                    return false;
                }
            }
        }
        list.push_back(w);
    }
}

}
