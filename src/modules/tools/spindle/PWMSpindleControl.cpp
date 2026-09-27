/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "libs/Kernel.h"
#include "PWMSpindleControl.h"
#include "SpindleConfig.h"
#include "Logging.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"
#include "SpindlePublicAccess.h"
#include "utils.h"

#include "libs/Pin.h"
#include "Gcode.h"
#include "GcodeDispatch.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"
#include "modules/robot/MachineTask.h"

#define UPDATE_FREQ 100


PWMSpindleControl *PWMSpindleControl::active = nullptr;

void PWMSpindleControl::configure(const void *c)
{
    const SpindleConfigT &s = *(const SpindleConfigT *)c;
    cfg.acc_ratio = s.acc_ratio;
    cfg.max_pwm = s.max_pwm;
    cfg.delay_s = s.delay_s;
    cfg.stall_count_rpm = s.stall_count_rpm;
    cfg.stall_alarm_rpm = s.stall_alarm_rpm;
    cfg.stall_s = s.stall_s;
}

void PWMSpindleControl::configure_active(const void *c)
{
    if (active) active->configure(c);
}

void PWMSpindleControl::on_module_loaded()
{
    const SpindleConfigT &spindle_config = spindle_cfg();
    configure(&spindle_config);
    last_time = 0;
    last_edge = 0;
    current_rpm = 0;
    current_I_value = 0;
    current_pwm_value = 0;
    time_since_update = 0;
    stall_timer = 0;

    spindle_on = false;

    factor = 100;

    pulses_per_rev = spindle_config.pulses_per_rev;
    target_rpm = spindle_config.default_rpm;
    control_P_term = spindle_config.control_p;
    control_I_term = spindle_config.control_i;
    control_D_term = spindle_config.control_d;

    alarm_pin.from_spec(spindle_config.alarm_pin)->as_input();

    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = spindle_config.control_smoothing;
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_spec(spindle_config.pwm_pin);
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }

    if (pwm_pin == NULL)
    {
        printk("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    int period = spindle_config.pwm_period;
    pwm_pin->period_us(period);
    pwm_pin->write(output_inverted ? 1 : 0);

    // Get the pin for interrupt
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_spec(spindle_config.feedback_pin);
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            feedback_pin = new mbed::InterruptIn(pinname);
            feedback_pin->rise(this, &PWMSpindleControl::on_pin_rise);
            NVIC_SetPriority(EINT3_IRQn, 16);
        } else {
            printk("Error: Spindle feedback pin has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    }

    spindle_speed_timer.setFrequency(UPDATE_FREQ);
	spindle_speed_timer.start();
    active = this;
}

void PWMSpindleControl::on_pin_rise()
{
	if (irq_count >= pulses_per_rev) {
		irq_count = 0;
		rev_count ++;
		uint32_t timestamp = us_ticker_read();
		rev_time = timestamp - last_rev_time;
		last_rev_time = timestamp;
		time_since_update = 0;
	}
	irq_count ++;
}

void PWMSpindleControl::on_update_speed()
{
    // the VFD latches its alarm output, so one read is the whole check
    if(!machine_task.is_halted() && alarm_pin.get()) {
        machine_task.halt(SPINDLE_ALARM, "spindle alarm, power off/on");
        return;
    }

    // If we don't get any interrupts for 1 second, set current RPM to 0
    if (++time_since_update > UPDATE_FREQ)
    {
    	current_rpm = 0;
    }
    else{    // Calculate current RPM

	    uint32_t t = rev_time;
	    if (t > 2000 * cfg.acc_ratio ) //RPM < 30000
	    {	
	        float new_rpm = 1000000 * cfg.acc_ratio * 60.0f / t;
	        current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
	    }
	}

    if (spindle_on) {
    	if (update_count > UPDATE_FREQ / 5) {
    		update_count = 0;
            float error = target_rpm * (factor / 100) - current_rpm;
//            current_I_value += control_I_term * error * 1.0f / UPDATE_FREQ;
//            current_I_value = confine(current_I_value, -1.0f, 1.0f);
            float acc_pwm = control_P_term * error;
//            acc_pwm += current_I_value;
//            acc_pwm += control_D_term * UPDATE_FREQ * (error - prev_error);
            float new_pwm = current_pwm_value + acc_pwm;
            new_pwm = confine(new_pwm, 0.0f, cfg.max_pwm);

            prev_error = error;
            current_pwm_value = new_pwm;
    	}
    	update_count ++;

    	/*
		float error = target_rpm * (factor / 100) - current_rpm;
		current_I_value += control_I_term * error * 1.0f / UPDATE_FREQ;
		current_I_value = confine(current_I_value, -1.0f, 1.0f);

        float new_pwm = 0.1f;
        new_pwm += control_P_term * error;
        new_pwm += current_I_value;
        new_pwm += control_D_term * UPDATE_FREQ * (error - prev_error);
        new_pwm = confine(new_pwm, 0.0f, 1.0f);
        prev_error = error;

        current_pwm_value = new_pwm;

        */

		if (current_pwm_value > cfg.max_pwm) {
			current_pwm_value = cfg.max_pwm;
		}
    } else {
        current_I_value = 0;
        current_pwm_value = 0;
    }

    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
}

// the wait is the spindle reaching speed, so it is a sleep and not a dwell in the path
void PWMSpindleControl::turn_on() {
    spindle_on = true;
    if (cfg.delay_s > 0) delay_ms(cfg.delay_s * 1000);
}

void PWMSpindleControl::kill() {
    spindle_on = false;
    current_pwm_value = 0;
    if(pwm_pin != nullptr) pwm_pin->write(output_inverted ? 1 : 0);
}

void PWMSpindleControl::turn_off() {
    spindle_on = false;
    if (cfg.delay_s > 0) delay_ms(cfg.delay_s * 1000);
}


void PWMSpindleControl::set_speed(int rpm) {
    target_rpm = rpm;
}


void PWMSpindleControl::report_speed() {
    printk("State: %s, Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f\n",
    			spindle_on ? "on" : "off", current_rpm, target_rpm, current_pwm_value);
}


void PWMSpindleControl::set_p_term(float p) {
    control_P_term = p;
}


void PWMSpindleControl::set_i_term(float i) {
    control_I_term = i;
}


void PWMSpindleControl::set_d_term(float d) {
    control_D_term = d;
}


void PWMSpindleControl::report_settings() {
    printk("P: %0.6f I: %0.6f D: %0.6f\n",
                               control_P_term, control_I_term, control_D_term);
}

void PWMSpindleControl::set_factor(float new_factor) {
	factor = new_factor;
}

// returns spindle status
void PWMSpindleControl::get_status(struct spindle_status *t)
{
    t->state = this->spindle_on;
    t->current_rpm = this->current_rpm;
    t->target_rpm = this->target_rpm;
    t->current_pwm_value = this->current_pwm_value;
    t->factor = this->factor;
}



// returns spindle status
// get stall status
bool PWMSpindleControl::get_stall(void)
{
	if (this->spindle_on && this->target_rpm > cfg.stall_count_rpm && this->current_rpm < cfg.stall_alarm_rpm) {
		if (stall_timer == 0) {
			stall_timer = us_ticker_read();
		} else if (us_ticker_read() - stall_timer > (uint32_t)cfg.stall_s * 1000000) {
			return true;
		}
	} else {
		stall_timer = 0;
	}
	return false;
}

