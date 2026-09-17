// Host test: c++ -std=c++11 -I../src/modules/communication/utils gcodeline_test.cpp ../src/modules/communication/utils/GcodeLine.cpp ../src/modules/communication/utils/Gcode.cpp
#include "GcodeLine.h"
#include "Gcode.h"

#include <cmath>
#include <cstdio>
#include <map>

class StreamOutput {};

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
#define NEAR(a, b) (std::fabs((a) - (b)) < 1e-4f)

struct Params : gcode::ParamStore {
    std::map<int, float> v;
    std::map<std::string, float> named;
    bool get(int n, float &out) const override {
        auto it = v.find(n);
        if (it == v.end()) return false;
        out = it->second;
        return true;
    }
    bool set(int n, float val) override { v[n] = val; return true; }
    bool get_named(const char *name, float &out) const override {
        auto it = named.find(name);
        if (it == named.end()) return false;
        out = it->second;
        return true;
    }
    bool set_named(const char *name, float val, std::string &) override { named[name] = val; return true; }
};

static const gcode::Word *word(const gcode::Line &l, char c) {
    for (const gcode::Word &w : l.words()) if (w.letter == c) return &w;
    return nullptr;
}

int main() {
    Params p;
    p.v[101] = 5; p.v[102] = 2;
    gcode::Line l;

    CHECK(l.parse("N10 G1 X5 Y-2.5 F100 ; move", &p));
    CHECK(l.words().size() == 4);
    CHECK(word(l, 'G') && word(l, 'G')->value == 1);
    CHECK(word(l, 'X') && NEAR(word(l, 'X')->value, 5));
    CHECK(word(l, 'Y') && NEAR(word(l, 'Y')->value, -2.5f));
    CHECK(!word(l, 'N'));

    CHECK(l.parse("G28.2 X Y", &p));
    CHECK(word(l, 'G')->subcode == 2 && word(l, 'X') && word(l, 'Y') && !word(l, 'X')->has_value && word(l, 'G')->has_value);

    CHECK(l.parse("g0 x1 (comment) y2", &p));
    CHECK(word(l, 'X') && word(l, 'Y'));

    CHECK(l.parse("G1 X5 (G91)", &p));
    CHECK(l.words().size() == 2);

    CHECK(l.parse("M3S1000", &p));
    CHECK(word(l, 'M')->value == 3 && word(l, 'S')->value == 1000);

    CHECK(l.parse("X#101 Y[#101*2+1] Z[-#102] A#[100+1]", &p));
    CHECK(NEAR(word(l, 'X')->value, 5));
    CHECK(NEAR(word(l, 'Y')->value, 11));
    CHECK(NEAR(word(l, 'Z')->value, -2));
    CHECK(NEAR(word(l, 'A')->value, 5));

    CHECK(l.parse("X[1+2*3] Y[[1+2]*3] Z[10/4]", &p));
    CHECK(NEAR(word(l, 'X')->value, 7));
    CHECK(NEAR(word(l, 'Y')->value, 9));
    CHECK(NEAR(word(l, 'Z')->value, 2.5f));

    CHECK(l.parse("%", &p) && l.words().empty());
    CHECK(l.parse("   ", &p) && l.words().empty());
    CHECK(l.parse("(only a comment)", &p) && l.words().empty());

    CHECK(!l.parse("X#999", &p) && l.error_text() == "no value for parameter #999");
    CHECK(!l.parse("X#101", nullptr));
    CHECK(!l.parse("X1 X2", &p) && l.error_text() == "duplicate word X");
    CHECK(!l.parse("X[1/0]", &p) && l.error_text() == "division by zero");
    CHECK(!l.parse("X[1+2", &p) && l.error_text() == "missing ]");
    CHECK(!l.parse("G0 X1*57", &p));
    CHECK(!l.parse("X#101+5", &p) && l.error_text() == "operators need brackets");
    CHECK(!l.parse("X1e-3", &p) && l.error_text() == "bad number");
    CHECK(!l.parse("X1E5", &p));
    CHECK(!l.parse("X1e", &p) && l.error_text() == "bad number");
    std::string signs = "X" + std::string(200, '-') + "1 Y" + std::string(201, '-') + "1";
    CHECK(l.parse(signs.c_str(), &p) && NEAR(word(l, 'X')->value, 1) && NEAR(word(l, 'Y')->value, -1));
    CHECK(!l.parse("X\xC3", &p) && l.error_text() == "unexpected byte 0xC3");
    CHECK(l.parse("X1 E5", &p) && word(l, 'E')->value == 5);
    CHECK(!l.parse("X[99999*99999*99999*99999*99999*99999*99999*99999]", &p) && l.error_text() == "value out of range");
    CHECK(!l.parse("X#[99999*99999*99999]", &p) && l.error_text() == "bad parameter number");
    CHECK(!l.parse("X#-1", &p));
    CHECK(l.parse("X-#101 Y+2", &p) && NEAR(word(l, 'X')->value, -5) && NEAR(word(l, 'Y')->value, 2));
    CHECK(!l.parse("G X1", &p) && l.error_text() == "bad code");
    CHECK(!l.parse("G28. X1", &p) && l.error_text() == "bad subcode");
    CHECK(l.parse("G1 X1 (unterminated", &p) && l.words().size() == 2);
    CHECK(!l.parse("G1 X1 )", &p));
    CHECK(!l.parse("X[[[[[[[[[[1]]]]]]]]]]", &p) && l.error_text() == "expression too deep");
    CHECK(l.parse("X[[[[1]]]]", &p));
    CHECK(!l.parse("X[[[[[1]]]]]", &p));
    CHECK(!l.parse("M99999999999", &p) && l.error_text() == "bad code");
    CHECK(!l.parse("G28.256", &p) && l.error_text() == "bad subcode");
    CHECK(l.parse("G1 G91 M3", &p) && l.words().size() == 3);

    const char *e = "1 + 2 * 3 - 4 / 2";
    float v; std::string err;
    CHECK(gcode::eval(e, v, &p, err) && NEAR(v, 5) && *e == 0);
    p.named["_clamp"] = 2; p.named["tool_x"] = -100.5f;
    struct { const char *text; float want; } exprs[] = {
        {"2 ** 3 ** 2", 64}, {"2 ** 3 * 2", 16}, {"7 MOD 3", 1}, {"1 + 2 EQ 3", 1}, {"1 + 2 EQ 4", 0},
        {"[1 EQ 1] AND [2 GT 1]", 1}, {"1 EQ 1 AND 2 GT 3", 0}, {"0 OR 5", 1}, {"1 XOR 1", 0}, {"3 NE 3", 0},
        {"2 LE 2", 1}, {"2 GE 3", 0}, {"#<_clamp> NE 2", 0}, {"#<tool_x> + 0.5", -100}, {"ABS[-3]", 3},
        {"SQRT[16]", 4}, {"SIN[30]", 0.5f}, {"COS[60]", 0.5f}, {"ATAN[1]/[1]", 45}, {"FIX[2.7]", 2}, {"FUP[2.2]", 3},
        {"ROUND[2.5]", 3}, {"FIX[-2.5]", -3}, {"EXISTS[#<_clamp>]", 1}, {"EXISTS[#<nope>]", 0}, {"-2 ** 2", 4}, // the sign belongs to the value, as in LinuxCNC
        {"10 - 2 - 3", 5}, {"2 * [3 + 4]", 14}, {"[#101 + 1] * 2", 12},
    };
    for (auto &x : exprs) {
        const char *q = x.text; float r;
        bool ok = gcode::eval(q, r, &p, err);
        if (!ok || !NEAR(r, x.want) || *q != 0) { printf("FAIL expr %s -> %g (%s) want %g rest '%s'\n", x.text, r, err.c_str(), x.want, q); failures++; }
    }
    CHECK(!l.parse("X[1 EQ]", &p));
    CHECK(!l.parse("X[FOO[1]]", &p) && l.error_text() == "unknown function FOO");
    CHECK(!l.parse("X[7 MOD 0]", &p) && l.error_text() == "division by zero");
    CHECK(!l.parse("X#<nope>", &p) && l.error_text() == "no value for parameter #<nope>");
    CHECK(l.parse("X#<TOOL_X> Y#<_clamp>", &p) && NEAR(word(l, 'X')->value, -100.5f) && word(l, 'Y')->value == 2);
    CHECK(l.parse("X[1 AND 0]", &p) && word(l, 'X')->value == 0);

    Gcode g1("G32 X1.2 Y2.3", nullptr);
    CHECK(g1.has_g && !g1.has_m && g1.g == 32 && g1.subcode == 0);
    CHECK(g1.get_num_args() == 2 && NEAR(g1.get_value('X'), 1.2f) && NEAR(g1.get_value('Y'), 2.3f));

    Gcode g2("M6 T3", nullptr);
    CHECK(g2.has_m && g2.m == 6 && g2.get_num_args() == 0 && g2.get_int('T') == 3);

    Gcode g3("G10 L20 P0 X-1.5", nullptr);
    CHECK(g3.get_int('L') == 20 && g3.get_uint('P') == 0 && g3.get_uint('X') == 0 && NEAR(g3.get_value('X'), -1.5f));
    Gcode g7("P4294967296 Q99999999999 R-99999999999", nullptr);
    CHECK(g7.get_uint('P') == 4294967295u && g7.get_int('Q') == 2147483647 && g7.get_int('R') == -2147483647 - 1);
    CHECK(g3.get_args().size() == 3 && g3.get_args().count('G') == 0);

    Gcode g4 = g3;
    CHECK(g4.g == 10 && g4.has_letter('X'));

    gcode::Line ml;
    CHECK(ml.parse("G90 G0 X1 M3 S100", &p));
    Gcode g5(ml.words(), 1, "G90 G0 X1 M3 S100", nullptr, 7);
    CHECK(g5.has_g && g5.g == 0 && !g5.has_m && g5.line == 7 && g5.has_letter('S') && g5.get_num_args() == 2);
    Gcode g6(ml.words(), ml.words().size(), "", nullptr, 0);
    CHECK(!g6.has_g && !g6.has_m);

    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
