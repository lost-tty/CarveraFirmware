/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "AnalogSpindleControl.h"
#include "SpindleConfig.h"
#include "Logging.h"
#include "PwmOut.h"

AnalogSpindleControl *AnalogSpindleControl::active = nullptr;

void AnalogSpindleControl::configure(const void *c)
{
    const SpindleConfigT &s = *(const SpindleConfigT *)c;
    cfg.max_rpm = s.max_rpm;
    cfg.min_rpm = s.min_rpm;
}

void AnalogSpindleControl::configure_active(const void *c)
{
    if (active) active->configure(c);
}

void AnalogSpindleControl::on_module_loaded()
{
    const SpindleConfigT &spindle_config = spindle_cfg();
    configure(&spindle_config);

    spindle_on = false;
    target_rpm = 0;

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_spec(spindle_config.pwm_pin);
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    // If we got no hardware PWM pin, delete this module
    if (pwm_pin == NULL)
    {
        printk("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    // set pwm frequency
    int period = spindle_config.pwm_period;
    pwm_pin->period_us(period);
    // invert pwm signal if necessary
    pwm_pin->write(output_inverted ? 1 : 0);

    // Get digital out pin for switching the VFD on and off (wired to a digital input on the VFD via an optocoupler)
    switch_on = NULL;
    if(PinSpec::connected(spindle_config.switch_on_pin)) {
        switch_on = new Pin();
        switch_on->from_spec(spindle_config.switch_on_pin)->as_output()->set(false);
    }
    active = this;
}

void AnalogSpindleControl::turn_on() 
{
    // set the output for switching the VFD on
    if(switch_on != NULL) 
        switch_on->set(true); 
    spindle_on = true;

}


void AnalogSpindleControl::kill()
{
    if(switch_on != nullptr) switch_on->set(false);
    spindle_on = false;
    if(pwm_pin != nullptr) update_pwm(0);
}

void AnalogSpindleControl::turn_off() 
{
    // clear the output for switching the VFD on 
    if(switch_on != NULL) 
        switch_on->set(false);
    spindle_on = false;
    // set the PWM value to 0 to make sure it stops
    update_pwm(0);

}


void AnalogSpindleControl::set_speed(int rpm) 
{
    // limit the requested RPM value
    if(rpm < 0) {
        target_rpm = 0;
    } else if (rpm > cfg.max_rpm) {
        target_rpm = cfg.max_rpm;
    } else if (rpm > 0 && rpm < cfg.min_rpm){
        target_rpm = cfg.min_rpm;
    } else {
        target_rpm = rpm;
    }
    // calculate the duty cycle and update the PWM
    update_pwm(1.0f / cfg.max_rpm * target_rpm);

}


void AnalogSpindleControl::report_speed()
{
    // report the current PWM value, calculate the current RPM value and report it as well
    float current_pwm = pwm_pin->read();
    printk("Current RPM: %.0f Analog value: %5.3f Target RPM: %d\n",
                               cfg.max_rpm * current_pwm, current_pwm, target_rpm);

}


void AnalogSpindleControl::update_pwm(float value) 
{
    // set the requested PWM value, invert it if necessary
    if(output_inverted)
        pwm_pin->write(1.0f - value);
    else
        pwm_pin->write(value);

}

