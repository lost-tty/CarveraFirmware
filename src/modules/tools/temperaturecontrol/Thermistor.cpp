/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Thermistor.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "SpindleTempConfig.h"
#include "libs/Median.h"
#include "utils.h"
#include "Logging.h"

// a const list of predefined thermistors
#include "predefined_thermistors.h"

#include <fastmath.h>

#include "MRI_Hooks.h"

#define UNDEFINED -1

Thermistor::Thermistor()
{
    this->bad_config = false;
    this->use_steinhart_hart= false;
    this->beta= 0.0F; // not used by default
    min_temp= 999;
    max_temp= 0;
    this->thermistor_number= 0; // not a predefined thermistor
}

Thermistor::~Thermistor()
{
}

// M305 may change these at runtime.
void Thermistor::UpdateConfig()
{
    const SpindleTempConfigT &spindle_temp_config = spindle_temp_cfg();
    this->r0   = spindle_temp_config.r0;
    this->t0   = spindle_temp_config.t0;
    this->r1   = spindle_temp_config.r1;
    this->r2   = spindle_temp_config.r2;
    this->beta = spindle_temp_config.beta;
    this->use_steinhart_hart= false;

    this->thermistor_pin.from_spec(spindle_temp_config.thermistor_pin);
    THEKERNEL->adc.enable_pin(&thermistor_pin);

    calc_jk();
}

// print out predefined thermistors
void Thermistor::print_predefined_thermistors(StreamOutput* s)
{
    int cnt= 1;
    s->printf("S/H table\n");
    for (auto& i : predefined_thermistors) {
        s->printf("%d - %s\n", cnt++, i.name);
    }

    cnt= 129;
    s->printf("Beta table\n");
    for (auto& i : predefined_thermistors_beta) {
        s->printf("%d - %s\n", cnt++, i.name);
    }
}

// calculate the coefficients from the supplied three Temp/Resistance pairs
// copied from https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
std::tuple<float,float,float> Thermistor::calculate_steinhart_hart_coefficients(float t1, float r1, float t2, float r2, float t3, float r3)
{
    float l1 = logf(r1);
    float l2 = logf(r2);
    float l3 = logf(r3);

    float y1 = 1.0F / (t1 + 273.15F);
    float y2 = 1.0F / (t2 + 273.15F);
    float y3 = 1.0F / (t3 + 273.15F);
    float x = (y2 - y1) / (l2 - l1);
    float y = (y3 - y1) / (l3 - l1);
    float c = (y - x) / ((l3 - l2) * (l1 + l2 + l3));
    float b = x - c * (powf(l1,2) + powf(l2,2) + l1 * l2);
    float a = y1 - (b + powf(l1,2) * c) * l1;

    if(c < 0) {
        printk("WARNING: negative coefficient in calculate_steinhart_hart_coefficients. Something may be wrong with the measurements\n");
        c = -c;
    }
    return std::make_tuple(a, b, c);
}

void Thermistor::calc_jk()
{
    // Thermistor math
    if(beta > 0.0F) {
        j = (1.0F / beta);
        k = (1.0F / (t0 + 273.15F));
    }else{
        printk("WARNING: beta cannot be 0\n");
        this->bad_config= true;
    }
}

float Thermistor::get_temperature()
{
    if(bad_config) return infinityf();

    int adc_value= new_thermistor_reading();
    if(adc_value == (int)Adc::not_ready) return infinityf();

    float t= adc_value_to_temperature(adc_value);
    // keep track of min/max for M305
    if(t > max_temp) max_temp= t;
    if(t < min_temp) min_temp= t;
    return t;
}

void Thermistor::get_raw()
{
    if(this->bad_config) {
       printk("WARNING: The config is bad for this temperature sensor\n");
    }

    int adc_value= new_thermistor_reading();
    const uint32_t max_adc_value= THEKERNEL->adc.get_max_value();

     // resistance of the thermistor in ohms
    float r = r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    printk("adc= %d, resistance= %f\n", adc_value, r);

    float t;
    if(this->use_steinhart_hart) {
        printk("S/H c1= %1.18f, c2= %1.18f, c3= %1.18f\n", c1, c2, c3);
        float l = logf(r);
        t= (1.0F / (this->c1 + this->c2 * l + this->c3 * powf(l,3))) - 273.15F;
        printk("S/H temp= %f, min= %f, max= %f, delta= %f\n", t, min_temp, max_temp, max_temp-min_temp);
    }else{
        t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
        printk("beta temp= %f, min= %f, max= %f, delta= %f\n", t, min_temp, max_temp, max_temp-min_temp);
    }

    // if using a predefined thermistor show its name and which table it is from
    if(thermistor_number != 0) {
        std::string name= (thermistor_number&0x80) ? predefined_thermistors_beta[(thermistor_number&0x7F)-1].name :  predefined_thermistors[thermistor_number-1].name;
        printk("Using predefined thermistor %d in %s table: %s\n", thermistor_number&0x7F, (thermistor_number&0x80)?"Beta":"S/H", name.c_str());
    }

    // reset the min/max
    min_temp= max_temp= t;
}

float Thermistor::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc.get_max_value();
    if ((adc_value >= max_adc_value) || (adc_value == 0))
        return infinityf();

    // resistance of the thermistor in ohms
    float r = r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    if(r > this->r0 * 8) return infinityf(); // 800k is probably open circuit

    float t;
    if(this->use_steinhart_hart) {
        float l = logf(r);
        t= (1.0F / (this->c1 + this->c2 * l + this->c3 * powf(l,3))) - 273.15F;
    }else{
        // use Beta value
        t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
    }

    return t;
}

int Thermistor::new_thermistor_reading()
{
    // filtering now done in ADC
    return THEKERNEL->adc.read(&thermistor_pin);
}

bool Thermistor::set_optional(const sensor_options_t& options) {
    bool define_beta= false;
    bool change_beta= false;
    uint8_t define_shh= 0;
    uint8_t predefined= 0;

    for(auto &i : options) {
        switch(i.first) {
            case 'B': this->beta= i.second; define_beta= true; break;
            case 'R': this->r0= i.second; change_beta= true; break;
            case 'X': this->t0= i.second; change_beta= true; break;
            case 'I': this->c1= i.second; define_shh++; break;
            case 'J': this->c2= i.second; define_shh++; break;
            case 'K': this->c3= i.second; define_shh++; break;
            case 'P': predefined= roundf(i.second); break;
        }
    }

    if(predefined != 0) {
        if(define_beta || change_beta || define_shh != 0) {
            // cannot use a predefined with any other option
            this->bad_config= true;
            return false;
        }

        if(predefined & 0x80) {
            // use the predefined beta table
            uint8_t n= (predefined&0x7F)-1;
            if(n >= sizeof(predefined_thermistors_beta) / sizeof(thermistor_beta_table_t)) {
                // not a valid index
                return false;
            }
            auto &i= predefined_thermistors_beta[n];
            this->beta = i.beta;
            this->r0 = i.r0;
            this->t0 = i.t0;
            this->r1 = i.r1;
            this->r2 = i.r2;
            use_steinhart_hart= false;
            calc_jk();
            thermistor_number= predefined;
            this->bad_config= false;
            return true;

        }else {
            // use the predefined S/H table
            uint8_t n= predefined-1;
            if(n >= sizeof(predefined_thermistors) / sizeof(thermistor_table_t)) {
                // not a valid index
                return false;
            }
            auto &i= predefined_thermistors[n];
            this->c1 = i.c1;
            this->c2 = i.c2;
            this->c3 = i.c3;
            this->r1 = i.r1;
            this->r2 = i.r2;
            use_steinhart_hart= true;
            thermistor_number= predefined;
            this->bad_config= false;
            return true;
        }
    }

    bool error= false;
    // if in Steinhart-Hart mode make sure B is specified, if in beta mode make sure all C1,C2,C3 are set and no beta settings
    // this is needed if swapping between modes
    if(use_steinhart_hart && define_shh == 0 && !define_beta) error= true; // if switching from SHH to beta need to specify new beta
    if(!use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if in beta mode and switching to SHH malke sure no beta settings are set
    if(!use_steinhart_hart && !(define_beta || change_beta) && define_shh != 3) error= true; // if in beta mode and switching to SHH must specify all three SHH
    if(use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if setting SHH anfd already in SHH do not specify any beta values

    if(error) {
        this->bad_config= true;
        return false;
    }
    if(define_beta || change_beta) {
        calc_jk();
        use_steinhart_hart= false;
    }else if(define_shh > 0) {
        use_steinhart_hart= true;
    }else{
        return false;
    }

    if(this->bad_config) this->bad_config= false;

    return true;
}

bool Thermistor::get_optional(sensor_options_t& options) {
    if(thermistor_number != 0) {
        options['P']= thermistor_number;
        return true;
    }

    if(use_steinhart_hart) {
        options['I']= this->c1;
        options['J']= this->c2;
        options['K']= this->c3;

    }else{
        options['B']= this->beta;
        options['X']= this->t0;
        options['R']= this->r0;
    }

    return true;
};
