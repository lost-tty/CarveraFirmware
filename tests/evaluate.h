#pragma once

// A script line as the dispatcher reads it: its parameters evaluated with the runner's, its words
// written back as text, and an error worded as the runner words its own.
#include "GcodeLine.h"
#include "Script.h"

#include <cstdio>
#include <string>

inline bool evaluate(const script::Runner &r, std::string &line, std::string &err)
{
    if (line[0] != '(' && line[0] != '%' && line.find('#') == std::string::npos
        && line.find('[') == std::string::npos)
        return true;

    gcode::Line l;
    if (!l.parse(line.c_str(), &r.parameters())) {
        char at[24];
        snprintf(at, sizeof(at), "line %u: ", r.last().line);
        err = at + l.error_text();
        return false;
    }
    line.clear();
    for (const gcode::Word &w : l.words()) {
        char buf[24];
        if (w.letter == 'G' || w.letter == 'M') {
            snprintf(buf, sizeof(buf), w.subcode ? "%c%d.%d" : "%c%d", w.letter, (int)w.value,
                     w.subcode);
        } else if (!w.has_value) {
            snprintf(buf, sizeof(buf), "%c", w.letter);
        } else {
            int n = snprintf(buf, sizeof(buf), "%c%.4f", w.letter, w.value);
            while (buf[n - 1] == '0') buf[--n] = 0;
            if (buf[n - 1] == '.') buf[--n] = 0;
        }
        if (!line.empty()) line += ' ';
        line += buf;
    }
    return true;
}
