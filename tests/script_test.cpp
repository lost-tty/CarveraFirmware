// Host test: c++ -std=c++11 -I../src/modules/communication/utils -I../src/modules/utils/script script_test.cpp ../src/modules/utils/script/Script.cpp ../src/modules/communication/utils/GcodeLine.cpp
#include "Script.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <set>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

struct Machine : gcode::ParamStore {
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
    std::set<std::string> readonly;
    bool set_named(const char *name, float val, std::string &) override {
        if (named.count(name) == 0 || readonly.count(name)) return false; // as Parameters does
        named[name] = val;
        return true;
    }
};

// runs a script (sub or main body) and returns the emitted lines joined by '|', or "ERROR: ..."
static std::string run(const char *text, Machine &m, const char *sub = nullptr, std::vector<float> args = {}) {
    script::Source src(text);
    script::Program prog;
    std::string err;
    if (!prog.load(src, err)) return "LOAD: " + err;
    script::Runner r(prog, m);
    if (!r.start(sub, args.data(), args.size(), err)) return "START: " + err;
    std::string out, line;
    for (int i = 0; i < 100000; i++) {
        script::Runner::Result res = r.step(line, err);
        if (res == script::Runner::DONE) {
            if (r.aborted() != 0) { char b[24]; snprintf(b, sizeof(b), "|ABORT %d", (int)r.aborted()); out += b; }
            return out;
        }
        if (res == script::Runner::ERROR) return out + (out.empty() ? "" : "|") + "ERROR: " + err;
        if (!out.empty()) out += '|';
        out += res == script::Runner::MESSAGE ? "[" + line + "]" : line;
    }
    return out + "|TIMEOUT";
}

static std::string load_error(const char *text) {
    script::Source src(text);
    script::Program prog;
    std::string err;
    return prog.load(src, err) ? "ok" : err;
}

int main() {
    Machine m;
    m.v[5021] = 12.5f;
    m.named["_clamp_state"] = 2;
    m.named["_tool_x"] = -100;

    CHECK(run("G21\nG90\nG0 X10\n", m) == "G21|G90|G0 X10");
    CHECK(run("(comment)\n\n; another\n%\nG0 X1\n", m) == "G0 X1");
    CHECK(run("G0 X#5021 Y[#5021 * 2]\n", m) == "G0 X12.5 Y25");
    CHECK(run("G53 G0 X#<_tool_x>\nG28.2 X Y\n", m) == "G53 G0 X-100|G28.2 X Y");

    const char *sub = "o<pick> sub\n  G0 X#1 Y#2\n  o<pick> return [#1 + #2]\no<pick> endsub\n";
    CHECK(run(sub, m, "pick", {3, 4}) == "G0 X3 Y4");
    CHECK(run("o<pick> sub\nG0 X#1\no<pick> endsub\nG1 Y1\no<pick> call [7]\nG1 Y2\n", m) == "G1 Y1|G0 X7|G1 Y2");
    CHECK(run("o<a> sub\n#<_r> = [#1 * 2]\no<a> endsub\no<a> call [21]\nG0 X#<_r>\n", m) == "G0 X42");
    CHECK(run("o<a> sub\no<a> return [5]\no<a> endsub\no<a> call\nG0 X#<_value> Y#<_value_returned>\n", m) == "G0 X5 Y1");
    CHECK(run("o<a> sub\nG0 X#2\no<a> endsub\no<a> call [1]\n", m) == "ERROR: line 2: no value for parameter #2");
    CHECK(run("o<a> sub\n#<x> = 3\nG0 X#<x>\no<a> endsub\no<a> call\nG0 Y#<x>\n", m) == "G0 X3|ERROR: line 6: no value for parameter #<x>");

    const char *branches = "#<_v> = %d\no100 if [#<_v> EQ 1]\nG0 X1\no100 elseif [#<_v> EQ 2]\nG0 X2\no100 else\nG0 X3\no100 endif\nG0 Y9\n";
    char buf[512];
    snprintf(buf, sizeof(buf), branches, 1); CHECK(run(buf, m) == "G0 X1|G0 Y9");
    snprintf(buf, sizeof(buf), branches, 2); CHECK(run(buf, m) == "G0 X2|G0 Y9");
    snprintf(buf, sizeof(buf), branches, 7); CHECK(run(buf, m) == "G0 X3|G0 Y9");
    CHECK(run("o1 if [0]\nG0 X1\no1 endif\nG0 Y2\n", m) == "G0 Y2");
    CHECK(run("o1 if [1]\no2 if [0]\nG0 X1\no2 else\nG0 X2\no2 endif\no1 else\nG0 X3\no1 endif\n", m) == "G0 X2");

    CHECK(run("#<_i> = 0\no1 while [#<_i> LT 3]\nG0 X#<_i>\n#<_i> = [#<_i> + 1]\no1 endwhile\n", m) == "G0 X0|G0 X1|G0 X2");
    CHECK(run("o1 repeat [3]\nG0 X1\no1 endrepeat\nG0 X2\n", m) == "G0 X1|G0 X1|G0 X1|G0 X2");
    CHECK(run("o1 repeat [0]\nG0 X1\no1 endrepeat\nG0 X2\n", m) == "G0 X2");
    CHECK(run("#<_i> = 0\no1 while [1]\n#<_i> = [#<_i> + 1]\no2 if [#<_i> GT 2]\no1 break\no2 endif\nG0 X#<_i>\no1 endwhile\nG0 Y0\n", m) == "G0 X1|G0 X2|G0 Y0");
    CHECK(run("#<_i> = 0\no1 while [#<_i> LT 4]\n#<_i> = [#<_i> + 1]\no2 if [#<_i> EQ 2]\no1 continue\no2 endif\nG0 X#<_i>\no1 endwhile\n", m) == "G0 X1|G0 X3|G0 X4");
    CHECK(run("o1 repeat [2]\no2 repeat [2]\nG0 X1\no2 endrepeat\no1 endrepeat\n", m) == "G0 X1|G0 X1|G0 X1|G0 X1");
    CHECK(run("o1 while [1]\no1 endwhile\n", m) == "ERROR: line 1: script does not progress");
    CHECK(run("o<a> sub\no<a> call\no<a> endsub\no<a> call\n", m) == "ERROR: line 2: call too deep");

    // the runner writes machine parameters through, and machine-owned names are read-only unless the machine takes them
    CHECK(run("#501 = 7\nG0 X#501\n", m) == "G0 X7" && m.v[501] == 7);
    CHECK(run("#<_clamp_state> = 5\nG0 X#<_clamp_state>\n", m) == "G0 X5" && m.named["_clamp_state"] == 5);

    const char *atc =
        "o<atc_pick> sub\n"
        "  M497.2\n"
        "  G53 G0 Z#<_atc_clearance_z>\n"
        "  G53 G0 X#<_tool_x> Y[#1 * 10]\n"
        "  M492.1\n"
        "  o100 if [#<_clamp_state> NE 2]\n"
        "    o<atc_pick> return [1]\n"
        "  o100 endif\n"
        "  M493.2 T#1\n"
        "o<atc_pick> endsub\n";
    m.named["_atc_clearance_z"] = -3;
    m.named["_clamp_state"] = 2;
    CHECK(run(atc, m, "atc_pick", {3}) == "M497.2|G53 G0 Z-3|G53 G0 X-100 Y30|M492.1|M493.2 T3");
    m.named["_clamp_state"] = 1;
    CHECK(run(atc, m, "atc_pick", {3}) == "M497.2|G53 G0 Z-3|G53 G0 X-100 Y30|M492.1");

    CHECK(load_error("o1 if [1]\nG0 X1\n") == "line 1: unclosed if");
    CHECK(load_error("o1 endif\n") == "line 1: no matching if");
    CHECK(load_error("o<a> sub\no<b> endsub\n") == "line 2: no matching sub");
    CHECK(load_error("o1 if [1]\no1 else\no1 elseif [1]\no1 endif\n") == "line 3: branch after else");
    CHECK(load_error("o1 while [1]\no2 break\no1 endwhile\n") == "line 2: no matching loop");
    CHECK(load_error("o<a> return\n") == "line 1: return outside sub");
    CHECK(load_error("o<a> call\n") == "line 1: call to unknown sub a");
    CHECK(load_error("o<a> sub\no<b> sub\no<b> endsub\no<a> endsub\n") == "line 2: sub inside sub");
    CHECK(load_error("o1 while [1]\no<a> sub\no1 break\no<a> endsub\no1 endwhile\n") == "line 2: sub inside while");
    CHECK(load_error("o1 if\no1 endif\n") == "line 1: expected one [condition]");
    CHECK(load_error("o1 endif [1]\n") == "line 1: unexpected argument");
    // review findings: comments after o-words, tabs, leading comments, numeric labels by value, stale repeat counters
    CHECK(run("o<a> sub (#1 = tool)\nG0 X#1\no<a> endsub ; end\no<a> call [1]\n", m) == "G0 X1");
    CHECK(run("#<_t>\t= 5\no<a>\tsub\nG0\tX#<_t>\no<a> endsub\no<a> call\n", m) == "G0 X5");
    CHECK(run("(c) G0 X1\n(only a comment)\nG0 X2\n", m) == "G0 X1|G0 X2");
    CHECK(run("o010 if [1]\nG0 X1\no10 endif\n", m) == "G0 X1");
    CHECK(run("o<a> sub\no<a> return\no<a> endsub\no<a> call\nG0 X#<_value> Y#<_value_returned>\n", m) == "G0 X0 Y0");
    CHECK(run("#<_n> = 0\no1 repeat [2]\no2 while [1]\no3 repeat [2]\nG0 X1\n#<_n> = [#<_n> + 1]\no4 if [#<_n> EQ 1]\no2 break\no4 endif\no3 endrepeat\no2 break\no2 endwhile\no1 endrepeat\n", m) == "G0 X1|G0 X1|G0 X1");
    CHECK(run("#<_n> = 0\no1 while [#<_n> LT 2]\n#<_n> = [#<_n> + 1]\no3 repeat [2]\nG0 X#<_n>\no5 if [#<_n> EQ 1]\no1 continue\no5 endif\no3 endrepeat\no1 endwhile\n", m) == "G0 X1|G0 X2|G0 X2");
    CHECK(run("#[100 + 1] = 4\nG0 X#101\n", m) == "G0 X4");
    m.named["_big"] = 1e30f;
    CHECK(run("G0 X#<_big>\n", m) == "ERROR: line 1: value out of range");
    m.v[101] = 6;
    m.named["_clamp_state"] = 2;
    CHECK(run("(MSG, hello #101)\n(DEBUG, x is #101 and #<_clamp_state>)\n(PRINT,#5021)\nG0 X1\n", m) == "[hello #101]|[x is 6 and 2]|[12.5]|G0 X1");
    CHECK(run("(debug, #<nope>)\n", m) == "ERROR: line 1: no value for parameter #<nope>");
    CHECK(run("(comment) G0 X2\n", m) == "G0 X2");
    // a failure return from an inner sub stops the script and survives the outer endsub
    CHECK(run("o<in> sub\no<in> abort [6]\no<in> endsub\no<out> sub\no<in> call\nG0 X1\no<out> endsub\no<out> call\n", m) == "|ABORT 6");
    CHECK(run("o<in> sub\no<in> return [4]\no<in> endsub\no<out> sub\no<in> call\nG0 X#<_value>\no<out> endsub\no<out> call\n", m) == "G0 X4");
    CHECK(load_error("o<a> sub\no<a> abort\no<a> endsub\n") == "line 2: expected one [condition]");
    CHECK(load_error("o<a> abort [1]\n") == "line 1: abort outside sub");
    // a read-only machine name must not be shadowed by a script global (the read would still see the machine)
    m.readonly.insert("_tool");
    m.named["_tool"] = 3;
    CHECK(run("#<_tool> = 9\nG0 X#<_tool>\n", m) == "ERROR: line 1: parameter is read-only");
    CHECK(run("#<_clamp_state> = 9\nG0 X#<_clamp_state>\n", m) == "G0 X9"); // writable machine name
    {
        std::string many = "#<_i> = 0\no1 while [#<_i> LT 20000]\n#<_i> = [#<_i> + 1]\n(MSG, tick)\no1 endwhile\nG0 X1\n";
        std::string out = run(many.c_str(), m);
        CHECK(out.find("does not progress") == std::string::npos && out.rfind("G0 X1") != std::string::npos);
    }
    // globals persist across start(), locals for a block's words
    {
        script::Source src("o<g81> sub\nG0 X#<x> Y#<y>\n#<_cnt> = [#<_cnt> + 1]\no<g81> endsub\n");
        script::Program prog; std::string err; CHECK(prog.load(src, err));
        script::Runner r(prog, m);
        std::string line, joined;
        m.named.erase("_cnt");
        for (int i = 1; i <= 2; i++) {
            CHECK(r.start("g81", nullptr, 0, err));
            r.set_local("x", i); r.set_local("y", 10 * i);
            if (i == 1) {
                CHECK(r.step(line, err) == script::Runner::LINE && line == "G0 X1 Y10");
                CHECK(r.step(line, err) == script::Runner::ERROR && err == "line 3: no value for parameter #<_cnt>");
            }
        }
        CHECK(r.start("g81", nullptr, 0, err)); r.set_local("x", 3); r.set_local("y", 30);
        CHECK(r.step(line, err) == script::Runner::LINE && line == "G0 X3 Y30");
        CHECK(r.step(line, err) == script::Runner::ERROR); // _cnt still undefined: globals persist but never got set
    }
    CHECK(load_error("o1 if [1 EQ]\no1 endif\n").rfind("line 1:", 0) == 0);
    CHECK(load_error("G0 X1 X2\n") == "line 1: duplicate word X");
    CHECK(load_error("#<x> 5\n") == "line 1: expected =");
    CHECK(load_error("o1 frob\n") == "line 1: unknown o-word frob");
    CHECK(load_error("O<A> SUB\nO<A> ENDSUB\nO<a> CALL\n") == "ok"); // case-insensitive

    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
