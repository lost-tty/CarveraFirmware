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

// A transfer's input: a client's bytes as they arrive, possibly in another task (the wifi
// task feeds them while it receives them).
class RxSink {
    public:
        virtual void take(const uint8_t *p, size_t n) = 0;
        // After each feed, in the same task: a short reply for the client (its length, 0 for
        // none), sent before settle() does what may take long.
        virtual size_t reply(uint8_t *out, size_t room) { return 0; }
        virtual void settle() {}
};

// Read as it goes out, maybe in another task, and left alone while busy. A piece may be asked
// for again, but never out of order.
class TxSource {
    public:
        virtual size_t size() const = 0;
        virtual void read(size_t at, uint8_t *p, size_t n) = 0;
        volatile bool busy = false;
};

// This is a base class for all StreamOutput objects.

class StreamOutput {
    public:
        StreamOutput(){}
        virtual ~StreamOutput(){}

        virtual int printf(const char *format, ...) __attribute__ ((format(printf, 2, 3)));
        virtual int vprintf(const char*, va_list);
        virtual void send(uint8_t type, const void *payload, size_t len);
        virtual int puts(const char* buf, int size = 0) = 0;

        virtual void puts_source(TxSource *s);

        // From attach_sink on, what the client sends goes to the sink, what the stream still
        // held for it first; after detach_sink no more reaches it. false: this stream cannot.
        virtual bool attach_sink(RxSink *s) { return false; }
        virtual void detach_sink() {}

        static void console_lock();
        static void console_unlock();

        static void lock_broadcast();
        static void unlock_broadcast();

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

        // a client frame a transfer found among its own, for the console behind the stream
        virtual void console_frame(uint8_t type, const uint8_t *p, uint16_t len) {}
        virtual const std::string &cwd() const;
        virtual void set_cwd(const std::string &path);
};

#endif
