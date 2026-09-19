/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include "libs/Module.h"
#include "Serial.h" // mbed.h lib
#include "libs/Kernel.h"
#include <vector>
#include <string>
using std::string;
#include "libs/RingBuffer.h"
#include "FrameConsole.h"


#define baud_rate_setting_checksum CHECKSUM("baud_rate")

class SerialConsole : public Module, public FrameConsole {
    public:
        SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate );

        void on_module_loaded();
        void on_serial_char_received();
        void on_main_loop(void * argument);

        int putc(int c);
        int getc(void);
        int puts(const char*, int size = 0);
        int gets(char** buf, int size = 0);
        bool ready();

    private:
        mbed::Serial* serial;
        static const int RX_RAW_BUF = 256;       // power of two, RingBuffer requires it
        RingBuffer<char,RX_RAW_BUF> rx_raw;
        char raw_chunk[32];                      // gets() hands out raw bytes from here
};

#endif
