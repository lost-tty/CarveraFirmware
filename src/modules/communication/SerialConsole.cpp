/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include "SimpleShell.h"
#include <stdarg.h>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/Logging.h"
#include "libs/DeferredWake.h"

// Serial reading module
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate )
{
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams.append_stream(this);
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	while (this->serial->readable()) {
		char c = this->serial->getc();
		if (rx_raw.size() < rx_raw.capacity()) rx_raw.push_back(c);
    }
    defer_wake(WAKE_MAIN);
}

void SerialConsole::service()
{
    // taken where the interrupt put them: by an upload's sink, else decoded
    while (rx_raw.size() > 0) {
        int tail = rx_raw.tail, head = rx_raw.head;
        int n = (head > tail ? head : RX_RAW_BUF) - tail;
        int taken = n;
        if (sink != nullptr) {
            sink->take((const uint8_t *)&rx_raw.buffer[tail], n);
            answer_sink();
        } else {
            taken = feed((const uint8_t *)&rx_raw.buffer[tail], n);
        }

        rx_raw.tail = (tail + taken) & (RX_RAW_BUF - 1);
        // a FILE_START stopped it: the rest waits for the upload its line starts
        if (taken < n)
            break;
    }
    pump();
}

int SerialConsole::puts(const char* s, int size)
{
    size_t n = size == 0 ? strlen(s) : size;
    for (size_t i = 0; i < n; ++i) {
        this->putc(s[i]);
    }
    return n;
}

void SerialConsole::answer_sink()
{
    uint8_t out[16];
    size_t n = sink->reply(out, sizeof(out));
    if (n != 0) {
        console_lock();
        puts((const char *)out, n);
        console_unlock();
    }
    sink->settle();
}

bool SerialConsole::attach_sink(RxSink *s)
{
    sink = s;
    return true;
}

void SerialConsole::detach_sink()
{
    sink = nullptr;
}

int SerialConsole::putc(int c)
{
    return this->serial->putc(c);
}

