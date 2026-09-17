#include "GcodeLine.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

namespace gcode {

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

static const int MAX_DEPTH = 4; // bracket nesting on the main-loop stack

static bool unary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth);
static bool expr(const char *&p, float &out, const ParamStore *params, std::string &err, int depth, int min_prec);
static bool primary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth);

static bool is_alpha(char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_';
}

bool named_param(const char *&p, std::string &name, std::string &err)
{
    if (*p++ != '<') {
        err = "bad parameter name";
        return false;
    }
    name.clear();
    while (*p && *p != '>') {
        if (*p != ' ') name += tolower((unsigned char)*p);
        p++;
    }
    if (*p != '>' || name.empty()) {
        err = "bad parameter name";
        return false;
    }
    p++;
    return true;
}

static float deg(float rad) { return rad * 180 / M_PI; }
static float rad(float deg) { return deg * M_PI / 180; }
static float sin_deg(float a) { return sinf(rad(a)); }
static float cos_deg(float a) { return cosf(rad(a)); }
static float tan_deg(float a) { return tanf(rad(a)); }
static float asin_deg(float a) { return deg(asinf(a)); }
static float acos_deg(float a) { return deg(acosf(a)); }

struct Fn { const char *name; float (*fn)(float); };
static const Fn FUNCTIONS[] = {
    {"ABS", fabsf}, {"SQRT", sqrtf}, {"EXP", expf}, {"LN", logf}, {"FIX", floorf}, {"FUP", ceilf}, {"ROUND", roundf},
    {"SIN", sin_deg}, {"COS", cos_deg}, {"TAN", tan_deg}, {"ASIN", asin_deg}, {"ACOS", acos_deg},
};

// FUNC[expr], ATAN[y]/[x], EXISTS[#<name>]; p is after the identifier
static bool function(const char *name, const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    skip_space(p);
    if (*p != '[') {
        err = std::string("unknown word ") + name;
        return false;
    }
    if (strcasecmp(name, "EXISTS") == 0) {
        p++;
        skip_space(p);
        std::string pname;
        if (*p != '#' || p[1] != '<') {
            err = "EXISTS needs #<name>";
            return false;
        }
        p++;
        if (!named_param(p, pname, err)) return false;
        skip_space(p);
        if (*p++ != ']') {
            err = "missing ]";
            return false;
        }
        out = params != nullptr && params->exists_named(pname.c_str()) ? 1 : 0;
        return true;
    }
    float a;
    if (!primary(p, a, params, err, depth)) return false; // the bracketed argument
    if (strcasecmp(name, "ATAN") == 0) {
        skip_space(p);
        if (*p++ != '/') {
            err = "ATAN needs [y]/[x]";
            return false;
        }
        float b;
        if (!primary(p, b, params, err, depth)) return false;
        out = deg(atan2f(a, b));
        return true;
    }
    for (const Fn &f : FUNCTIONS) {
        if (strcasecmp(name, f.name) == 0) {
            out = f.fn(a);
            return true;
        }
    }
    err = std::string("unknown function ") + name;
    return false;
}

static bool primary(const char *&p, float &out, const ParamStore *params, std::string &err, int depth)
{
    if (depth > MAX_DEPTH) {
        err = "expression too deep";
        return false;
    }
    skip_space(p);
    if (*p == '[') {
        p++;
        if (!expr(p, out, params, err, depth + 1, 0)) return false;
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
        if (*p == '<') {
            std::string name;
            if (!named_param(p, name, err)) return false;
            if (params == nullptr || !params->get_named(name.c_str(), out)) {
                err = "no value for parameter #<" + name + ">";
                return false;
            }
            return true;
        }
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
    if (is_alpha(*p)) {
        char name[8];
        unsigned n = 0;
        while (is_alpha(*p) || is_digit(*p)) {
            if (n < sizeof(name) - 1) name[n++] = *p;
            p++;
        }
        name[n] = 0;
        return function(name, p, out, params, err, depth);
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

// binary operators by precedence: 1 AND OR XOR, 2 EQ NE GT GE LT LE, 3 + -, 4 * / MOD, 5 **
static float op_mul(float a, float b) { return a * b; }
static float op_div(float a, float b) { return a / b; }
static float op_add(float a, float b) { return a + b; }
static float op_sub(float a, float b) { return a - b; }
static float op_eq(float a, float b) { return a == b; }
static float op_ne(float a, float b) { return a != b; }
static float op_gt(float a, float b) { return a > b; }
static float op_ge(float a, float b) { return a >= b; }
static float op_lt(float a, float b) { return a < b; }
static float op_le(float a, float b) { return a <= b; }
static float op_and(float a, float b) { return a != 0 && b != 0; }
static float op_or(float a, float b) { return a != 0 || b != 0; }
static float op_xor(float a, float b) { return (a != 0) != (b != 0); }

struct Op { const char *text; uint8_t len; uint8_t prec; float (*fn)(float, float); };
static const Op OPS[] = {
    {"**", 2, 5, powf}, {"*", 1, 4, op_mul}, {"/", 1, 4, op_div}, {"MOD", 3, 4, fmodf}, {"+", 1, 3, op_add}, {"-", 1, 3, op_sub},
    {"EQ", 2, 2, op_eq}, {"NE", 2, 2, op_ne}, {"GT", 2, 2, op_gt}, {"GE", 2, 2, op_ge}, {"LT", 2, 2, op_lt}, {"LE", 2, 2, op_le},
    {"AND", 3, 1, op_and}, {"OR", 2, 1, op_or}, {"XOR", 3, 1, op_xor},
};

static const Op *peek_op(const char *p)
{
    for (const Op &op : OPS) {
        if (strncasecmp(p, op.text, op.len) != 0) continue;
        if (is_alpha(op.text[0]) && (is_alpha(p[op.len]) || is_digit(p[op.len]))) continue; // ANDX is not AND
        return &op;
    }
    return nullptr;
}

// precedence climbing: one stack frame per bracket level and per precedence step, not per operator
static bool expr(const char *&p, float &out, const ParamStore *params, std::string &err, int depth, int min_prec)
{
    if (!unary(p, out, params, err, depth)) return false;
    for (;;) {
        skip_space(p);
        const Op *op = peek_op(p);
        if (op == nullptr || op->prec < min_prec) return true;
        p += op->len;
        float rhs;
        if (!expr(p, rhs, params, err, depth, op->prec + 1)) return false;
        if (rhs == 0 && (op->text[0] == '/' || op->text[0] == 'M')) {
            err = "division by zero";
            return false;
        }
        out = op->fn(out, rhs);
    }
}

bool eval(const char *&p, float &out, const ParamStore *params, std::string &err)
{
    if (!expr(p, out, params, err, 0, 0)) return false;
    if (!std::isfinite(out)) {
        err = "value out of range";
        return false;
    }
    return true;
}

bool assign(const char *p, ParamStore &store, std::string &err)
{
    if (*p++ != '#') {
        err = "expected #";
        return false;
    }
    std::string name;
    float index = 0;
    if (*p == '<') {
        if (!named_param(p, name, err)) return false;
    } else if (!unary(p, index, &store, err, 1) || index < 0 || index != (int)index) {
        if (err.empty()) err = "bad parameter number";
        return false;
    }
    skip_space(p);
    if (*p++ != '=') {
        err = "expected =";
        return false;
    }
    float v;
    if (!eval(p, v, &store, err)) return false;
    skip_space(p);
    if (*p && *p != ';' && *p != '(') {
        err = "trailing characters";
        return false;
    }
    err.clear();
    bool ok = name.empty() ? store.set((int)index, v) : store.set_named(name.c_str(), v, err);
    if (!ok && err.empty()) err = "parameter is read-only";
    return ok;
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
