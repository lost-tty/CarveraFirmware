/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUT_H
#define STREAMOUTPUT_H

#include <cstdarg>
#include <cstring>
#include <cstdint>
#include <stdio.h>
#include <string>

// This is a base class for all StreamOutput objects.

class StreamOutput {
    public:
        StreamOutput(){}
        virtual ~StreamOutput(){}

        virtual int printf(const char *format, ...) __attribute__ ((format(printf, 2, 3)));
        virtual int vprintf(const char*, va_list);
        virtual void send(uint8_t type, const void *payload, size_t len);
        virtual int gets(char** buf, int size = 0) { return 0; }
        virtual int puts(const char* buf, int size = 0) = 0;
        virtual bool ready() { return true; };

        static void console_lock();
        static void console_unlock();

        static void lock_broadcast();
        static void unlock_broadcast();

        // set for the duration of a file transfer, which reads the stream's bytes itself
        virtual void set_transferring(bool) {}
        virtual bool is_transferring() const { return false; }
        virtual bool accept_event() const { return !is_transferring(); }

        // Collects INFO text sent to s into as few frames as possible. They are sent when the
        // Gather is destroyed, a frame is full, or any other output is sent.
        class Gather {
            public:
                explicit Gather(StreamOutput *s);
                ~Gather();
                Gather(const Gather &) = delete;
                Gather &operator=(const Gather &) = delete;
            private:
                StreamOutput *prev;
        };
        // call before writing with puts(): the text gathered so far goes out first
        static void flush_gathered();
        virtual const std::string &cwd() const;
        virtual void set_cwd(const std::string &path);
};

#endif
