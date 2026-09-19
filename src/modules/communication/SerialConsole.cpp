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
    : decoder(rx_frame, sizeof(rx_frame))
{
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
    this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams.append_stream(this);
}

// one frame per call: its command may start a transfer, and the bytes behind it are then payload
void SerialConsole::decode_rx() {
    while (rx_raw.size() > 0) {
        char c;
        rx_raw.pop_front(c);
        if (decoder.feed(c)) {
            on_frame();
            return;
        }
    }
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	while (this->serial->readable()) {
		char c = this->serial->getc();
		if (rx_raw.size() < rx_raw.capacity()) rx_raw.push_back(c);
    }
}

void SerialConsole::on_frame() {
    const uint8_t *p = decoder.payload();
    uint16_t len = decoder.length();

    switch (decoder.type()) {
        case Frame::CTRL_SINGLE:
            if (len < 1) return;
            switch (p[0]) {
                case '?': query_flag = true; break;
                case '*': diagnose_flag = true; break;
                case 'X' - 'A' + 1: halt_flag = true; break; // ^X
                case '!': if (THEKERNEL->is_feed_hold_enabled()) THEKERNEL->set_feed_hold(true); break;
                case '~': if (THEKERNEL->is_feed_hold_enabled()) THEKERNEL->set_feed_hold(false); break;
            }
            break;

        case Frame::CTRL_MULTI:
        case Frame::FILE_START: {
            if ((int)len + 1 > buffer.capacity() - buffer.size()) return;
            char last = '\n';
            for (uint16_t i = 0; i < len; i++) {
                char c = p[i] == '\r' ? '\n' : p[i];
                if (c == '\n' && last == '\n') continue;
                buffer.push_back(c);
                last = c;
            }
            if (last != '\n') buffer.push_back('\n');
            break;
        }

        default:
            break;
    }
}

void SerialConsole::on_idle(void * argument)
{
	if (THEKERNEL->is_uploading()) return;

    if (query_flag ) {
        query_flag = false;
        std::string s = THEKERNEL->get_query_string();
        send(Frame::STATUS, s.data(), s.size());
    }

    if (diagnose_flag) {
    	diagnose_flag = false;
        std::string s = THEKERNEL->get_diagnose_string();
    	send(Frame::DIAG, s.data(), s.size());
    }

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(MANUAL);
        printf("ALARM: Abort during cycle\r\n");
    }
}

void SerialConsole::on_main_loop(void * argument){
    decode_rx();
    if ( this->has_char('\n') ){
        string received;
        received.reserve(20);
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                SimpleShell::run(received, this);
                return;
            }else{
                received += c;
            }
        }
    }
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

bool SerialConsole::has_char(char letter){
    int index = this->buffer.tail;
    while( index != this->buffer.head ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}
