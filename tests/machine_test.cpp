// Host test of the machine script against the sequences the C++ ATC generators produced.
// c++ -std=c++11 -I../src/modules/communication/utils -I../src/modules/utils/script machine_test.cpp ../src/modules/utils/script/Script.cpp ../src/modules/communication/utils/GcodeLine.cpp
#include "Script.h"

#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <algorithm>
#include <map>
#include <string>
#include <vector>


static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

// the Carvera defaults from config.default and ATCHandler::on_config_reload
struct Machine : gcode::ParamStore {
    std::map<int, float> v{{5021, 100}, {5022, 50}, {5023, -20}};
    std::map<std::string, float> named{
        {"_anchor1_x", -359}, {"_anchor1_y", -234}, {"_anchor2_offset_x", 90}, {"_anchor2_offset_y", 45.65f},
        {"_toolrack_offset_x", 356}, {"_toolrack_offset_y", 0}, {"_toolrack_z", -105},
        {"_rotation_offset_x", -8}, {"_rotation_offset_y", 37.5f}, {"_rotation_offset_z", 22.5f},
        {"_clearance_x", -75}, {"_clearance_y", -3}, {"_clearance_z", -3},
        {"_atc_safe_z", -10}, {"_atc_safe_z_empty", -20}, {"_atc_safe_z_offset", 10},
        {"_atc_fast_z_rate", 500}, {"_atc_slow_z_rate", 60}, {"_atc_margin_rate", 1000},
        {"_atc_probe_fast_rate", 300}, {"_atc_probe_slow_rate", 60}, {"_atc_probe_retract", 2}, {"_atc_probe_height", 0},
        {"_probe_mx", -3}, {"_probe_my", -54}, {"_probe_mz", -145},
        {"_active_tool", -1}, {"_laser_mode", 0}, {"_spindle_on", 0}, {"_tlo", 10}, {"_clamp_state", 1}, {"_tool_detected", 0},
    };
    bool get(int n, float &out) const override { auto i = v.find(n); if (i == v.end()) return false; out = i->second; return true; }
    bool set(int n, float val) override { v[n] = val; return true; }
    bool get_named(const char *name, float &out) const override { auto i = named.find(name); if (i == named.end()) return false; out = i->second; return true; }
    bool set_named(const char *, float, std::string &) override { return false; }
};

static std::string text, messages;
static script::Program program;

struct Word { char letter; float value; };

// runs several subs on one runner, the way the module does, so #<_globals> persist between them
struct Call { const char *sub; std::vector<Word> words; int code; };
static std::string run_all(Machine &m, const std::vector<Call> &calls) {
    script::Runner r(program, m);
    std::string err, out, line;
    for (const Call &c : calls) {
        if (!r.start(c.sub, nullptr, 0, err)) return "START: " + err;
        r.set_local("subcode", 0);
        if (c.code) r.set_local("code", c.code);
        for (const Word &w : c.words) { char n[2] = {(char)tolower(w.letter), 0}; r.set_local(n, w.value); }
        out += "/";
        for (int i = 0; i < 10000; i++) {
            script::Runner::Result res = r.step(line, err);
            if (res == script::Runner::DONE) {
                if (r.aborted() != 0) { char b[24]; snprintf(b, sizeof(b), "ABORT %d", (int)r.aborted()); out += b; }
                break;
            }
            if (res == script::Runner::ERROR) return out + "ERROR: " + err;
            if (res == script::Runner::MESSAGE) continue;
            out += (out.back() == '/' ? "" : "|") + line;
        }
    }
    return out;
}

// runs a sub the way the Scripts module triggers it; `hook` sees each line and may change the machine
static std::string run(Machine &m, const char *sub, std::vector<float> args, std::vector<Word> words = {}, int subcode = 0,
                       void (*hook)(Machine &, const std::string &) = nullptr, int code = 0) {
    script::Runner r(program, m);
    std::string err, out, line;
    if (!r.start(sub, args.data(), args.size(), err)) return "START: " + err;
    r.set_local("subcode", subcode);
    if (code) r.set_local("code", code);
    for (Word &w : words) { char n[2] = {(char)tolower(w.letter), 0}; r.set_local(n, w.value); }
    for (int i = 0; i < 10000; i++) {
        script::Runner::Result res = r.step(line, err);
        if (res == script::Runner::DONE) {
            if (r.aborted() != 0) { char b[24]; snprintf(b, sizeof(b), "|ABORT %d", (int)r.aborted()); out += b; }
            return out;
        }
        if (res == script::Runner::ERROR) return out + "|ERROR: " + err;
        if (res == script::Runner::MESSAGE) { messages += line + "\n"; continue; }
        if (hook) hook(m, line);
        out += (out.empty() ? "" : "|") + line;
    }
    return out + "|TIMEOUT";
}

static std::string drop(int tool) {
    char b[300];
    int y = -234 + (tool == 0 ? 210 : (6 - tool) * 30);
    snprintf(b, sizeof(b), "M497.1|G23|G53 G0 Z-3|G53 G0 X-3 Y%d|M492.2|G53 G0 X-3 Y%d|G53 G1 Z-95 F500|G53 G1 Z-105 F60|M490.2|G53 G0 Z-20|M493.2 T-1|M492.1", y, y);
    return b;
}
static std::string pick(int tool, bool from_clearance) {
    char b[300];
    int y = -234 + (tool == 0 ? 210 : (6 - tool) * 30);
    snprintf(b, sizeof(b), "M497.2|G23|G53 G0 Z%d|G53 G0 X-3 Y%d|M492.1|M490.2|G53 G0 X-3 Y%d|G53 G1 Z-95 F500|G53 G1 Z-105 F60|M490.1|G53 G0 Z-10|M492.2", from_clearance ? -3 : -20, y, y);
    return b;
}
static std::string cali(int lift_z, bool probe, bool laser = false) {
    char b[300];
    snprintf(b, sizeof(b), "M497.3|G23|%sG53 G0 Z%d|G53 G0 X-3 Y-54|G38.6 Z-145 F300|G91 G0 Z2|G38.6 Z-3 F60|M493.1|G53 G0 Z-10%s", laser ? "M490.1|" : "", lift_z, probe ? "|M492.3" : "");
    return b;
}
static const std::string home = "G53 G0 Z-3|G53 G0 X100 Y50";

static void tlo_after_measure(Machine &m, const std::string &line) { if (line == "M493.1") m.named["_tlo"] = 10.2f; }

int main() {
    DIR *d = opendir("../src/macros");
    if (d == nullptr) { printf("run from tests/\n"); return 1; }
    std::vector<std::string> files;
    while (struct dirent *e = readdir(d)) if (strstr(e->d_name, ".ngc")) files.push_back(e->d_name);
    closedir(d);
    std::sort(files.begin(), files.end());
    for (const std::string &name : files) {
        FILE *f = fopen(("../src/macros/" + name).c_str(), "r");
        text += "(file: " + name + ")\n";
        for (int c; (c = fgetc(f)) != EOF;) text += (char)c;
        fclose(f);
        text += '\n';
    }
    script::Source source(text.c_str());
    std::string err;
    CHECK(program.load(source, err));
    if (!err.empty()) printf("%s\n", err.c_str());
    Machine m;

    // M6 from an empty spindle, to a tool, to the probe, to empty, same tool, invalid
    CHECK(run(m, "tool_change", {}, {{'T', 3}}) == "M5|" + pick(3, true) + "|" + cali(-10, false) + "|M493.2 T3|" + home);
    CHECK(messages == "tool change T-1 -> T3\n");
    CHECK(run(m, "tool_change", {}, {{'T', 0}}) == "M5|" + pick(0, true) + "|" + cali(-10, true) + "|M493.2 T0|" + home);
    m.named["_active_tool"] = 2;
    CHECK(run(m, "tool_change", {}, {{'T', -1}}) == "M5|" + drop(2) + "|" + home);
    CHECK(run(m, "tool_change", {}, {{'T', 3}}) == "M5|" + drop(2) + "|" + pick(3, false) + "|" + cali(-10, false) + "|M493.2 T3|" + home);
    CHECK(run(m, "tool_change", {}, {{'T', 2}}) == "M5");
    CHECK(run(m, "tool_change", {}, {{'T', 7}}) == "|ABORT 6");
    m.named["_spindle_on"] = 1;
    CHECK(run(m, "tool_change", {}, {{'T', 3}}) == "M5|ABORT 5");
    m.named["_spindle_on"] = 0;
    // laser mode: no picking, dropping calibrates the empty (clamped) spindle, M6 T-1 again re-measures
    m.named["_laser_mode"] = 1;
    CHECK(run(m, "tool_change", {}, {{'T', 3}}) == "M5");
    CHECK(run(m, "tool_change", {}, {{'T', -1}}) == "M5|" + drop(2) + "|" + cali(-10, false, true) + "|" + home);
    m.named["_active_tool"] = -1;
    CHECK(run(m, "tool_change", {}, {{'T', -1}}) == "M5|" + cali(-3, false, true) + "|" + home);
    m.named["_laser_mode"] = 0;

    // M491 measures; M491.1 checks against the stored length
    m.named["_active_tool"] = 2;
    CHECK(run(m, "calibrate", {}) == cali(-3, false) + "|" + home);
    CHECK(run(m, "calibrate", {}, {{'H', 0.05f}}, 1, tlo_after_measure) == cali(-3, false) + "|M5|G53 G0 Z-10|ABORT 4");
    m.named["_tlo"] = 10;
    CHECK(run(m, "calibrate", {}, {{'H', 0.5f}}, 1, tlo_after_measure) == cali(-3, false) + "|M5|G53 G0 Z-10|" + home);
    CHECK(run(m, "calibrate", {}, {{'H', 0.01f}}, 1) == "|ABORT 4");

    CHECK(run(m, "goto", {}, {}, 0) == "G53 G0 Z-3|G53 G0 X-75 Y-3");
    CHECK(run(m, "goto", {}, {}, 2) == "G53 G0 Z-3|G90 G0 X0 Y0");
    CHECK(run(m, "goto", {}, {}, 3) == "G53 G0 Z-3|G53 G0 X-359 Y-234");
    CHECK(run(m, "goto", {}, {}, 4) == "G53 G0 Z-3|G53 G0 X-269 Y-188.35");
    CHECK(run(m, "goto", {}, {{'X', 10}, {'Y', 20}}, 5) == "G53 G0 Z-3|G90 G0 X10 Y20");
    CHECK(run(m, "goto", {}, {{'X', 10}, {'Y', 20}}, 6) == "G53 G0 Z-3|G53 G0 X10 Y20");
    CHECK(run(m, "goto", {}, {}, 5) == "G53 G0 Z-3");
    CHECK(run(m, "g28", {}) == "G53 G0 Z-3|G53 G0 X-75 Y-3");

    // M495: probe tool first, then margin, z probe, leveling, origin
    m.named["_active_tool"] = 2;
    std::string probe_tool = drop(2) + "|" + pick(0, false) + "|" + cali(-10, true) + "|M493.2 T0";
    CHECK(run(m, "auto_work", {}, {{'X', 10}, {'Y', 20}, {'C', 50}, {'D', 60}}) == probe_tool + "|M497.4|M494.1|G53 G0 Z-3|G90 G0 X10 Y20|G90 G1 X10 Y60 F1000|G90 G1 X50 Y60 F1000|G90 G1 X50 Y20 F1000|G90 G1 X10 Y20 F1000|M494.2");
    m.named["_active_tool"] = 0;
    CHECK(run(m, "auto_work", {}, {{'X', 10}, {'Y', 20}, {'O', 5}, {'F', 6}, {'P', 0}}) == "M497.5|G53 G0 Z-3|G90 G0 X15 Y26|G38.2 Z-145 F300|G91 G0 Z2|G38.2 Z-3 F60|G10 L20 P0 Z0|G91 G0 Z2|G53 G0 Z-3|G90 G0 X10 Y20");
    CHECK(run(m, "auto_work", {}, {{'X', 10}, {'Y', 20}, {'O', 5}}) == "M497.5|G53 G0 Z-3|G53 G0 X-370 Y-196.5|G38.2 Z-145 F300|G91 G0 Z2|G38.2 Z-3 F60|G10 L20 P0 Z22.5|G91 G0 Z2");
    CHECK(run(m, "auto_work", {}, {{'X', 10}, {'Y', 20}, {'A', 100}, {'B', 80}, {'I', 5}, {'J', 4}, {'H', 5}}) == "M497.6|G90 G0 X10 Y20|G32 R1 X0 Y0 A100 B80 I5 J4 H5");
    CHECK(run(m, "auto_work", {}, {{'X', 10}, {'Y', 20}, {'P', 0}}) == "G53 G0 Z-3|G90 G0 X10 Y20");
    CHECK(run(m, "auto_work", {}, {{'X', 10}}) == "");
    CHECK(run(m, "auto_work", {}, {{'D', 6}, {'H', 12}}, 3) == "M497.5|G38.2 Z-145 F60|G10 L20 P0 Z12|G91 G0 Z2|G38.2 X-35 F60|G10 L20 P0 X3|G91 G0 X5|G38.2 Y-35 F60|G10 L20 P0 Y3|G91 G0 Y5|G91 G0 Z15|G91 G0 X-8 Y-8");
    CHECK(run(m, "auto_work", {}, {}, 3) == "M497.5|G38.2 Z-145 F60|G10 L20 P0 Z9|G91 G0 Z2|G38.2 X-35 F60|G10 L20 P0 X1.5875|G91 G0 X5|G38.2 Y-35 F60|G10 L20 P0 Y1.5875|G91 G0 Y5|G91 G0 Z15|G91 G0 X-6.5875 Y-6.5875");

    // hooks and demo macros
    CHECK(run(m, "before_resume", {10, 20, -5}) == "G90 G0 X10 Y20|G90 G1 Z-5 F1000");
    CHECK(run(m, "test_square", {10, 500, 1}) == "G91|G1 X10 F500|G1 Y10|G1 X-10|G1 Y-10|G90");
    CHECK(run(m, "test_circle", {5, 600, 2}) == "G91 G0 X-5|G2 X0 Y0 I5 J0 F600|G2 X0 Y0 I5 J0 F600|G91 G0 X5|G90");
    CHECK(run(m, "test_jog", {3, 400, 1}) == "G91|G1 X3 F400|G1 X-3|G90");

    // laser mode switching: mode first, then the tool goes back, then the offset
    m.named["_active_tool"] = 2;
    CHECK(run(m, "laser_on", {}) == "M321.2|M5|" + drop(2) + "|" + home + "|G92.5 Z0");
    CHECK(run(m, "laser_off", {}) == "M322.2|G92.1");
    m.named["_laser_mode"] = 1;
    CHECK(run(m, "laser_on", {}) == "");
    m.named["_laser_mode"] = 0;

    // canned cycles: one runner, as the module keeps it, so a cycle's sticky Z R F survive between holes
    m.v[5043] = 3; // work Z, where the cycle starts and G98 retracts to
    CHECK(run_all(m, {{"drill", {{'X', 10}, {'Y', 10}, {'Z', -2}, {'R', 1}, {'F', 100}}, 81},
                      {"drill", {{'X', 20}}, 81},
                      {"drill_cancel", {}, 0}})
          == "/G0 Z3|G0 X10 Y10|G0 Z1|G1 Z-2 F100|G0 Z3"
             "/G0 Z3|G0 X20|G0 Z1|G1 Z-2 F100|G0 Z3"
             "/G0 Z3");
    CHECK(run_all(m, {{"drill", {{'X', 10}, {'Z', -2}, {'R', 1}, {'F', 100}}, 81},
                      {"drill_retract_r", {}, 0},
                      {"drill", {{'X', 20}}, 81},
                      {"drill_retract_z", {}, 0},
                      {"drill", {{'X', 30}}, 81}})
          == "/G0 Z3|G0 X10|G0 Z1|G1 Z-2 F100|G0 Z3"
             "/"
             "/G0 Z3|G0 X20|G0 Z1|G1 Z-2 F100|G0 Z1"
             "/"
             "/G0 Z3|G0 X30|G0 Z1|G1 Z-2 F100|G0 Z3");
    CHECK(run_all(m, {{"drill", {{'X', 40}, {'Z', -2}, {'R', 1}, {'F', 100}, {'P', 2}}, 82}})
          == "/G0 Z3|G0 X40|G0 Z1|G1 Z-2 F100|G4 P2|G0 Z3");
    CHECK(run_all(m, {{"drill", {{'X', 50}, {'Z', -5}, {'R', 1}, {'F', 100}, {'Q', 2}}, 83}})
          == "/G0 Z3|G0 X50|G0 Z1|G1 Z-1 F100|G0 Z1|G1 Z-3 F100|G0 Z1|G1 Z-5 F100|G0 Z3");
    CHECK(run_all(m, {{"drill_cancel", {}, 0}}) == "/");
    CHECK(run_all(m, {{"drill", {{'X', 10}, {'R', 1}, {'F', 100}}, 81}}) == "/ABORT 17");
    CHECK(run_all(m, {{"drill", {{'X', 10}, {'Z', -2}, {'R', 1}}, 81}}) == "/ABORT 17");

    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
