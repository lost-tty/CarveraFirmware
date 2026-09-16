/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
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
    raw_mode = false;
	this->attach_irq(true);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams.append_stream(this);
}

// enable_irq == false: raw mode for a file transfer. The ISR stays attached because the UART FIFO
// is 16 bytes; it stores bytes in the ring buffer for gets().
void SerialConsole::attach_irq(bool enable_irq) {
	__disable_irq();
	buffer.tail = buffer.head;   // drop whatever the other mode left behind
	raw_mode = !enable_irq;
	decoder.reset();
	__enable_irq();
	this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);
}

void SerialConsole::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_serial_rx_irq_checksum)) {
        bool enable_irq = *static_cast<bool *>(pdr->get_data_ptr());
        this->attach_irq(enable_irq);
        pdr->set_taken();
    }
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	while (this->serial->readable()) {
		char c = this->serial->getc();
		if (raw_mode) {
			if (buffer.capacity() - (buffer.head - buffer.tail + ((buffer.tail > buffer.head) ? RX_LINE_BUF : 0)) > 0) {
				buffer.push_back(c);
			}
		} else if (decoder.feed(c)) {
			on_frame();
		}
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
            int room = buffer.capacity() - (buffer.head - buffer.tail + ((buffer.tail > buffer.head) ? RX_LINE_BUF : 0));
            if (len + 1 > room) return;
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

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    if (raw_mode) return; // the ring holds transfer data, not command lines
    if ( this->has_char('\n') ){
        string received;
        received.reserve(20);
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                message.line = 0;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
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
	if (raw_mode) {
		int n = 0;
		while (n < (int)sizeof(raw_chunk) && buffer.size() > 0) {
			buffer.pop_front(raw_chunk[n++]);
		}
		*buf = raw_chunk;
		return n;
	}
	getc_result = this->getc();
	*buf = &getc_result;
	return 1;
}

int SerialConsole::putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::getc()
{
    if (raw_mode) {
        char c = 0;
        if (buffer.size() == 0) return -1;
        buffer.pop_front(c);
        return (uint8_t)c;
    }
    return this->serial->getc();
}

bool SerialConsole::ready()
{
    if (raw_mode) return buffer.size() > 0;
    return this->serial->readable();
}

// Does the queue have a given char ?
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
