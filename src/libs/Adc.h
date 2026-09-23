/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef ADC_H
#define ADC_H

#include "PinNames.h" // mbed.h lib


class Pin;
namespace mbed {
    class ADC;
}

// define how many bits of extra resolution required
// 2 bits means the 12bit ADC is 14 bits of resolution
#define OVERSAMPLE 2

class Adc
{
public:
    void init();
    void enable_pin(Pin *pin);
    unsigned int read(Pin *pin);

    void new_sample(int chan, uint32_t value);
    // return the maximum ADC value, base is 12bits 4095.
    int get_max_value() const { return 4095 << OVERSAMPLE;}

    // read() answers this until the channel has averaged its first samples
    static const unsigned int not_ready= 0xFFFFFFFF;

private:
    PinName _pin_to_pinname(Pin *pin);
    mbed::ADC *adc;

    static const int num_channels= 6;
    static const int num_samples= 5;   // odd, so the median is one of the samples

    // A median, not an average: a spindle starting up puts spikes on the thermistor line, and
    // an average carries a share of every one of them into the reading. The median ignores
    // them outright as long as they are the minority.
    uint16_t samples[num_channels][num_samples];
    uint8_t at[num_channels];       // where the next sample goes
    uint8_t filled[num_channels];   // read() has nothing to say until the first ones are in
};

#endif
