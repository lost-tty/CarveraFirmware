/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Adc.h"
#include "libs/nuts_bolts.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Logging.h"
#include "libs/ADC/adc.h"
#include "libs/Pin.h"


#include "mbed.h"

// This is an interface to the mbed.org ADC library you can find in libs/ADC/adc.h
// TODO : Having the same name is confusing, should change that
Adc* instance;

void sample_isr(int chan, uint32_t value)
{
    instance->new_sample(chan, value);
}

void Adc::init()
{
    instance = this;
    // ADC sample rate need to be fast enough to be able to read the enabled channels within the thermistor poll time
    const uint32_t sample_rate= 1000; // 1KHz sample rate
    this->adc = new mbed::ADC(sample_rate, 8);
    this->adc->append(sample_isr);
}

/*
LPC176x ADC channels and pins

Adc Channel Port Pin    Pin Functions                       Associated PINSEL Register
AD0 P0.23   0-GPIO,     1-AD0[0], 2-I2SRX_CLK, 3-CAP3[0]    14,15 bits of PINSEL1
AD1 P0.24   0-GPIO,     1-AD0[1], 2-I2SRX_WS, 3-CAP3[1]     16,17 bits of PINSEL1
AD2 P0.25   0-GPIO,     1-AD0[2], 2-I2SRX_SDA, 3-TXD3       18,19 bits of PINSEL1
AD3 P0.26   0-GPIO,     1-AD0[3], 2-AOUT, 3-RXD3            20,21 bits of PINSEL1
AD4 P1.30   0-GPIO,     1-VBUS, 2- , 3-AD0[4]               28,29 bits of PINSEL3
AD5 P1.31   0-GPIO,     1-SCK1, 2- , 3-AD0[5]               30,31 bits of PINSEL3
AD6 P0.3    0-GPIO,     1-RXD0, 2-AD0[6], 3-                6,7 bits of PINSEL0
AD7 P0.2    0-GPIO,     1-TXD0, 2-AD0[7], 3-                4,5 bits of PINSEL0
*/

// Enables ADC on a given pin
void Adc::enable_pin(Pin *pin)
{
    PinName pin_name = this->_pin_to_pinname(pin);
    int channel = adc->_pin_to_channel(pin_name);
    if(channel < 0 || channel >= num_channels) {
        printk("ERROR: %d.%d cannot be read by the ADC\n", pin->port_number, pin->pin);
        return;
    }

    at[channel] = 0;
    filled[channel] = 0;

    this->adc->burst(1);
    this->adc->setup(pin_name, 1);
    this->adc->interrupt_state(pin_name, 1);
}

// the average is carried at the oversampled scale, so a 12 bit sample moves up first
// the ring is written here and sorted in read(): this runs in the ADC interrupt
void Adc::new_sample(int chan, uint32_t value)
{
    if(chan >= num_channels) return;

    samples[chan][at[chan]] = ((value >> 4) & 0xFFF) << OVERSAMPLE;
    at[chan] = (at[chan] + 1) % num_samples;
    if(filled[chan] < num_samples) filled[chan]++;
}

unsigned int Adc::read(Pin *pin)
{
    PinName p = this->_pin_to_pinname(pin);
    int channel = adc->_pin_to_channel(p);
    if(channel < 0 || channel >= num_channels) return not_ready;

    uint16_t sorted[num_samples];
    __disable_irq();
    bool ready = filled[channel] >= num_samples;
    for (int i = 0; i < num_samples; ++i) sorted[i] = samples[channel][i];
    __enable_irq();
    if(!ready) return not_ready;

    // five entries, so an insertion sort is smaller and faster than anything cleverer
    for (int i = 1; i < num_samples; ++i) {
        uint16_t v = sorted[i];
        int j = i;
        while(j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; j--; }
        sorted[j] = v;
    }
    return sorted[num_samples / 2];
}

// Convert a smoothie Pin into a mBed Pin
PinName Adc::_pin_to_pinname(Pin *pin)
{
    if( pin->port == LPC_GPIO0 && pin->pin == 23 ) {
        return p15;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 24 ) {
        return p16;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 25 ) {
        return p17;
    } else if( pin->port == LPC_GPIO0 && pin->pin == 26 ) {
        return p18;
    } else if( pin->port == LPC_GPIO1 && pin->pin == 30 ) {
        return p19;
    } else if( pin->port == LPC_GPIO1 && pin->pin == 31 ) {
        return p20;
    } else {
        //TODO: Error
        return NC;
    }
}

