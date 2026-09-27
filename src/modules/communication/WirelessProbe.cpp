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
#include "GcodeDispatch.h"
#include "Gcode.h"
#include "libs/nuts_bolts.h"
#include "WirelessProbe.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/Logging.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "SwitchPool.h"
#include "ConfigTable.h"

#define WIRELESSPROBE_CONFIG(X) \
    X(float, min_voltage, "min_voltage", 3.6f) \
    X(float, max_voltage, "max_voltage", 4.1f)
CONFIG_STRUCT(WirelessProbeConfig, WIRELESSPROBE_CONFIG);
CONFIG_KEYS(wp_config_keys, WirelessProbeConfig, WIRELESSPROBE_CONFIG);

#define UART_CONFIG(X) \
    X(float, baud_rate, "baud_rate", 115200.0f)
CONFIG_STRUCT(UartConfig, UART_CONFIG);
CONFIG_KEYS(uart_config_keys, UartConfig, UART_CONFIG);
extern WirelessProbe wireless_probe;
static void wp_config_changed(const ConfigTable::Group *, const void *c)
{
    wireless_probe.configure(c);
}
CONFIG_GROUPS(wireless_probe_config_groups,
    CFG_GROUP("wp", wp_config_keys, WirelessProbeConfig, wp_config_changed),
    CFG_GROUP("uart", uart_config_keys, UartConfig, nullptr));

void WirelessProbe::configure(const void *cfg)
{
    const WirelessProbeConfig &c = *(const WirelessProbeConfig *)cfg;
    this->min_voltage = c.min_voltage;
    this->max_voltage = c.max_voltage;
}



// Wireless probe serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it

// Called when the module has just been loaded
void WirelessProbe::on_module_loaded() {
    this->wp_voltage = 0.0;

	this->serial = new mbed::Serial( USBTX, USBRX );
    configure(&ConfigTable::config<WirelessProbeConfig>(wireless_probe_config_groups));
    this->serial->baud(ConfigTable::config<UartConfig>(&wireless_probe_config_groups[1]).baud_rate);

    // We want to be called every time a new char is received
    this->serial->attach(this, &WirelessProbe::on_serial_char_received, mbed::Serial::RxIrq);


    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    ADD_MCODE(m470, 470, IMMEDIATE, WirelessProbe::set_address);
    ADD_MCODE(m471, 471, IMMEDIATE, WirelessProbe::pair);
    ADD_MCODE(m472, 472, IMMEDIATE, WirelessProbe::laser_on);
    ADD_MCODE(m881, 881, IMMEDIATE, WirelessProbe::set_channel);
    ADD_MCODE(m882, 882, IMMEDIATE, WirelessProbe::stop_transmission);
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void WirelessProbe::on_serial_char_received() {
    while (this->serial->readable()){
        char received = this->serial->getc();
        // convert CR to NL (for host OSs that don't send NL)
        if ( received == '\r' ) { received = '\n'; }
        this->buffer.push_back(received);
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void WirelessProbe::on_main_loop(void * argument) {
    if ( this->has_char('\n') ) {
        string received;
        received.reserve(20);
        while (1) {
           char c;
           this->buffer.pop_front(c);
           if ( c == '\n' ) {
        	   // printk("WP received: [%s]\n", received.c_str());
        	   if (received[0] == 'V') {
            	   // get wireless probe voltage
            	   Gcode gc(received, &StreamOutput::NullStream);
            	   if (gc.get_value('V') <= 4.2) {
                	   this->wp_voltage = gc.get_value('V');
                	   // compare voltage value and switch probe charger
                	   if (this->wp_voltage <= this->min_voltage) {
                		   struct pad_switch pad;
                           bool ok = SwitchPool::get_state(probecharger_checksum, &pad);
                           if (!ok || !pad.state) {
                        	   printk("WP voltage: [%1.2fV], start charging\n", this->wp_voltage);
                    		   bool b = true;
                    		   SwitchPool::set_state(probecharger_checksum, b);
                           }
                	   } else if (this->wp_voltage >= this->max_voltage) {
                		   struct pad_switch pad;
                           bool ok = SwitchPool::get_state(probecharger_checksum, &pad);
                           if (!ok || pad.state) {
                        	   printk("WP voltage: [%1.2fV], end charging\n", this->wp_voltage);
                    		   bool b = false;
                    		   SwitchPool::set_state(probecharger_checksum, b);
                           }
                	   }
            	   }
        	   } else if (received[0] == 'A' && received.length() > 2) {
        		   // get wireless probe address
        		   uint16_t probe_addr = ((uint16_t)received[2] << 8) | received[1];
        		   printk("WP power: [%1.2fv], addr: [%0d]\n", this->wp_voltage, probe_addr);
        	   } else if (received[0] == 'P' && received.length() > 1) {
        		   printk("WP PAIR %s!\n", received[1] ? "SUCCESS" : "TIMEOUT");
        	   }
               return;
            } else {
                received += c;
            }
        }
    }
}

int WirelessProbe::puts(const char* s)
{
    //return fwrite(s, strlen(s), 1, (FILE*)(*this->serial));
    size_t n= strlen(s);
    for (size_t i = 0; i < n; ++i) {
        putc(s[i]);
    }
    return n;
}

int WirelessProbe::gets(char** buf)
{
	getc_result = this->getc();
	*buf = &getc_result;
	return 1;
}

int WirelessProbe::putc(int c)
{
    return this->serial->putc(c);
}

int WirelessProbe::getc()
{
    return this->serial->getc();
}

// Does the queue have a given char ?
bool WirelessProbe::has_char(char letter){
    int index = this->buffer.tail;
    while( index != this->buffer.head ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}



void WirelessProbe::set_address(Gcode *gcode)
{
    if(!gcode->has_letter('S')) return;
    uint16_t new_addr = gcode->get_value('S');
    printk("Change WP address to: [%d]\n", new_addr);
    this->putc('S');
    this->putc(new_addr & 0xff);
    this->putc(new_addr >> 8);
    this->putc('#');
}

void WirelessProbe::pair(Gcode *gcode)
{
    printk("Set WP into pairing mode...\n");
    this->putc('P');
}

void WirelessProbe::laser_on(Gcode *gcode)
{
    printk("Open WP Laser...\n");
    this->putc('L');
}

void WirelessProbe::set_channel(Gcode *gcode)
{
    if(!gcode->has_letter('S')) return;
    uint16_t channel = gcode->get_value('S');
    printk("Set 2.4G Channel to: [%d] and start trans...\n", channel);
    this->putc(channel);
}

void WirelessProbe::stop_transmission(Gcode *gcode)
{
    printk("Stop 2.4G transmission...\n");
    this->putc(27);
}
