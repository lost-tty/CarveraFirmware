/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Gcode.h"

#include <climits>

Gcode::Gcode(const string& text, StreamOutput* stream, unsigned int line)
    : m(0), g(0), line(line), subcode(0), has_m(false), has_g(false), mcs(false), stream(stream)
{
    gcode::Line parsed;
    if (!parsed.parse(text.c_str(), nullptr)) {
        error_text = parsed.error_text();
        return;
    }
    words = parsed.words();
    // module-built lines carry a single command, take the first G and the first M
    for (const gcode::Word& w : words) {
        if ((w.letter == 'G' && !has_g) || (w.letter == 'M' && !has_m)) set_command(w);
    }
}

Gcode::Gcode(const gcode::Words& words, size_t command, StreamOutput* stream, unsigned int line)
    : m(0), g(0), line(line), subcode(0), has_m(false), has_g(false), mcs(false), stream(stream), words(words)
{
    if (command < words.size()) set_command(words[command]);
}

void Gcode::set_command(const gcode::Word& w)
{
    if (w.letter == 'G') {
        has_g = true;
        g = w.value;
    } else {
        has_m = true;
        m = w.value;
    }
    subcode = w.subcode;
}

const gcode::Word* Gcode::find(char letter) const
{
    for (const gcode::Word& w : words) {
        if (w.letter == letter) return &w;
    }
    return nullptr;
}

float Gcode::get_value(char letter) const
{
    const gcode::Word* w = find(letter);
    return w ? w->value : 0;
}

int Gcode::get_int(char letter) const
{
    float v = get_value(letter);
    if (v <= (float)INT_MIN) return INT_MIN;
    if (v >= (float)INT_MAX) return INT_MAX;
    return (int)v;
}

uint32_t Gcode::get_uint(char letter) const
{
    float v = get_value(letter);
    if (v <= 0) return 0;
    if (v >= (float)UINT32_MAX) return UINT32_MAX;
    return (uint32_t)v;
}

int Gcode::get_num_args() const
{
    int n = 0;
    for (const gcode::Word& w : words) {
        if (is_parameter(w.letter)) n++;
    }
    return n;
}

std::map<char,float> Gcode::get_args() const
{
    std::map<char,float> args;
    for (const gcode::Word& w : words) {
        if (is_parameter(w.letter)) args[w.letter] = w.value;
    }
    return args;
}
