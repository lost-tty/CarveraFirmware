/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"
#include "utils.h"
#include "FileTransfer.h"
#include "Configurator.h"
#include "SoftTimer.h"

#include <functional>
#include <string>
#include <cstdint>

#include "FreeRTOS.h"
#include "timers.h"

using std::string;

class StreamOutput;

class SimpleShell : public Module
{
public:
    template<class T> struct Sub {
        const char *name;
        void (T::*fn)(std::string args, StreamOutput *stream);
        const char *help;
    };

    template<class T> static void dispatch(T *self, const Sub<T> *subs, const char *cmd, std::string args, StreamOutput *stream)
    {
        std::string what = shift_parameter(args);
        for (const Sub<T> *s = subs; s->fn != nullptr; ++s) {
            if (what == s->name) {
                (self->*(s->fn))(args, stream);
                return;
            }
        }
        for (const Sub<T> *s = subs; s->fn != nullptr; ++s) {
            stream->printf("%s %s - %s\r\n", cmd, s->name, s->help);
        }
    }

    typedef void (*command_fn)(void *context, const char *name, std::string args, StreamOutput *stream);
    static const std::string &cwd() { return current_path; }

    struct Registered { const char *name; command_fn command; void *context; const char *help; Registered *next; };
    static void add_command(Registered &slot, const char *name, command_fn fn, void *context, const char *help);
    static void run(const std::string &line, StreamOutput *stream);
    static bool control_char(char c, StreamOutput *stream);
    void run_command(const std::string &line, StreamOutput *stream);
    SimpleShell()
    : resetTimer("SimpleShell::resetTimer", 3000, false, this, &SimpleShell::system_reset_callback)
    {}

    void on_module_loaded();
    bool parse_command(const char *cmd, string args, StreamOutput *stream);
    void print_mem(StreamOutput *stream) { mem_command("", stream); }
    void version_command(string parameters, StreamOutput *stream );
        void model_command(std::string parameters, StreamOutput *stream );
    void ftype_command( string parameters, StreamOutput *stream );

private:

    void jog(string params, StreamOutput *stream);

    void ls_command(string parameters, StreamOutput *stream );
    void cd_command(string parameters, StreamOutput *stream );
    void delete_file_command(string parameters, StreamOutput *stream );
    void pwd_command(string parameters, StreamOutput *stream );
    void upload_command(string parameters, StreamOutput *stream);
    void download_command(string parameters, StreamOutput *stream);
    void compute_md5sum_command(string parameters, StreamOutput *stream);
    void cat_command(string parameters, StreamOutput *stream );
    void echo_command(string parameters, StreamOutput *stream );
    void rm_command(string parameters, StreamOutput *stream );
    void mv_command(string parameters, StreamOutput *stream );
    void mkdir_command(string parameters, StreamOutput *stream );
    void break_command(string parameters, StreamOutput *stream );
    void reset_command(string parameters, StreamOutput *stream );
    void dfu_command(string parameters, StreamOutput *stream );
    void help_command(string parameters, StreamOutput *stream );
    void get_command(string parameters, StreamOutput *stream );
    void calc_thermistor_command( string parameters, StreamOutput *stream);
    void print_thermistors_command( string parameters, StreamOutput *stream);
    void md5sum_command( string parameters, StreamOutput *stream);
    void grblDP_command( string parameters, StreamOutput *stream);

    void switch_command(string parameters, StreamOutput *stream);
    void mem_command(string parameters, StreamOutput *stream);
    void task_command(string parameters, StreamOutput *stream);

    void net_command( string parameters, StreamOutput *stream);
    void ap_command( string parameters, StreamOutput *stream);
    void wlan_command( string parameters, StreamOutput *stream);
    void diagnose_command( string parameters, StreamOutput *stream);
    void sleep_command( string parameters, StreamOutput *stream);
    void power_command( string parameters, StreamOutput *stream);

    void remount_command( string parameters, StreamOutput *stream);

    void test_command( string parameters, StreamOutput *stream);

    void time_command( string parameters, StreamOutput *stream);

    void config_get_all_command(string parameters, StreamOutput *stream );

    void config_restore_command(string parameters, StreamOutput *stream );

    void config_default_command(string parameters, StreamOutput *stream );
    void eeprom_command(string parameters, StreamOutput *stream );
    void eeprom_show(string parameters, StreamOutput *stream );
    void eeprom_clear(string parameters, StreamOutput *stream );
    static const Sub<SimpleShell> EEPROM_SUBS[];

    void system_reset_callback();

    typedef void (*PFUNC)(string parameters, StreamOutput *stream);
    typedef struct {
        const char* name;
        void (SimpleShell::*command)(std::string parameters, StreamOutput* stream);
        const char *help;
    } ptentry_t;

    static const ptentry_t commands_table[];

    static Registered *registered;
    static std::string current_path;

    FileTransfer transfer;
    Configurator      configurator;

    SoftTimer resetTimer;
};
