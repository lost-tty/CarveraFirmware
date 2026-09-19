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
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"

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

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams.append_stream(this);
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	while (this->serial->readable()) {
		char c = this->serial->getc();
		if (rx_raw.size() < rx_raw.capacity()) rx_raw.push_back(c);
    }
}

void SerialConsole::on_main_loop(void * argument){
    decode(rx_raw);
    string line;
    if (next_line(line)) SimpleShell::run(line, this);
}

int SerialConsole::puts(const char* s, int size)
{
    size_t n = size == 0 ? strlen(s) : size;
    for (size_t i = 0; i < n; ++i) {
        this->putc(s[i]);
    }
    return n;
}

int SerialConsole::gets(char** buf, int size)
{
	int n = 0;
	while (n < (int)sizeof(raw_chunk) && rx_raw.size() > 0) {
		rx_raw.pop_front(raw_chunk[n++]);
	}
	*buf = raw_chunk;
	return n;
}

int SerialConsole::putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::getc()
{
    char c = 0;
    if (rx_raw.size() == 0) return -1;
    rx_raw.pop_front(c);
    return (uint8_t)c;
}

bool SerialConsole::ready()
{
    return rx_raw.size() > 0;
}

