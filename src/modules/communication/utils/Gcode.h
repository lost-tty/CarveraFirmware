/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GCODE_H
#define GCODE_H

#include "GcodeLine.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

using std::string;

class StreamOutput;

// One command word (G or M) of a line plus all of the line's parameter words
class Gcode {
    public:
        Gcode(const string& text, StreamOutput* stream, unsigned int line = 0);
        // command is an index into words, or words.size() for a line without G or M
        Gcode(const gcode::Words& words, size_t command, const string& text, StreamOutput* stream, unsigned int line);

        const char* get_command() const { return text.c_str(); }
        bool has_letter(char letter) const { return find(letter) != nullptr; }
        float get_value(char letter) const;
        int get_int(char letter) const;
        uint32_t get_uint(char letter) const;
        int get_num_args() const;
        std::map<char,float> get_args() const;
        const gcode::Words& get_words() const { return words; }

        unsigned int m;
        unsigned int g;
        unsigned int line;
        uint8_t subcode;

        struct {
            bool add_nl:1;
            bool has_m:1;
            bool has_g:1;
            bool is_error:1;
            bool mcs:1;                                       // G53: this motion is in machine coordinates
        };

        StreamOutput* stream;
        string txt_after_ok;

    private:
        const gcode::Word* find(char letter) const;
        void set_command(const gcode::Word& w);
        static bool is_parameter(char letter) { return letter != 'G' && letter != 'M' && letter != 'T'; }

        gcode::Words words;
        string text;
};

#endif
