#include "libs/Kernel.h"
#include "MainButton.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Logging.h"
#include "Endstops.h"
#include "Player.h"
#include "SwitchPublicAccess.h"
#include "SwitchPool.h"
#include "modules/robot/MachineTask.h"

using namespace std;

#define main_button_enable_checksum         CHECKSUM("main_button_enable")
#define main_button_pin_checksum            CHECKSUM("main_button_pin")
#define main_button_LED_R_pin_checksum      CHECKSUM("main_button_LED_R_pin")
#define main_button_LED_G_pin_checksum      CHECKSUM("main_button_LED_G_pin")
#define main_button_LED_B_pin_checksum      CHECKSUM("main_button_LED_B_pin")
#define main_button_poll_frequency_checksum CHECKSUM("main_button_poll_frequency")
#define main_long_press_time_ms_checksum    CHECKSUM("main_button_long_press_time")
#define main_button_long_press_checksum     CHECKSUM("main_button_long_press_enable")

#define e_stop_pin_checksum                 CHECKSUM("e_stop_pin")
#define ps12_pin_checksum                   CHECKSUM("ps12_pin")
#define ps24_pin_checksum                   CHECKSUM("ps24_pin")
#define power_fan_delay_s_checksum          CHECKSUM("power_fan_delay_s")

#define power_checksum                      CHECKSUM("power")
#define auto_sleep_checksum                 CHECKSUM("auto_sleep")
#define auto_sleep_min_checksum             CHECKSUM("auto_sleep_min")
#define turn_off_min_checksum               CHECKSUM("turn_off_min")
#define stop_on_cover_open_checksum         CHECKSUM("stop_on_cover_open")



void MainButton::on_module_loaded()
{
    this->using_12v = false;
    this->led_update_timer = 0;
    this->second_counter = 0;
    this->hold_toggle = 0;
    this->button_state = NONE;
    this->button_pressed = false;
    this->stop_on_cover_open = false;
    this->sleep_countdown_us = us_ticker_read();
    this->light_countdown_us = us_ticker_read();
    this->power_fan_countdown_us = us_ticker_read();

    bool main_button_enable = THEKERNEL->config->value( main_button_enable_checksum )->by_default(true)->as_bool(); // @deprecated
    if (!main_button_enable) {
        return;
    }

    this->main_button.from_string( THEKERNEL->config->value( main_button_pin_checksum )->by_default("1.16^")->as_string())->as_input();
    this->main_button_LED_R.from_string( THEKERNEL->config->value( main_button_LED_R_pin_checksum )->by_default("1.10")->as_string())->as_output();
    this->main_button_LED_G.from_string( THEKERNEL->config->value( main_button_LED_G_pin_checksum )->by_default("1.15")->as_string())->as_output();
    this->main_button_LED_B.from_string( THEKERNEL->config->value( main_button_LED_B_pin_checksum )->by_default("1.14")->as_string())->as_output();
    this->poll_frequency = THEKERNEL->config->value( main_button_poll_frequency_checksum )->by_default(20)->as_number();
    this->long_press_time_ms = THEKERNEL->config->value( main_long_press_time_ms_checksum )->by_default(3000)->as_number();
    this->long_press_enable = THEKERNEL->config->value( main_button_long_press_checksum )->by_default(false)->as_string();

    this->e_stop.from_string( THEKERNEL->config->value( e_stop_pin_checksum )->by_default("0.26^")->as_string())->as_input();
    this->PS12.from_string( THEKERNEL->config->value( ps12_pin_checksum )->by_default("0.22")->as_string())->as_output();
    this->PS24.from_string( THEKERNEL->config->value( ps24_pin_checksum )->by_default("0.10")->as_string())->as_output();
    this->power_fan_delay_s = THEKERNEL->config->value( power_fan_delay_s_checksum )->by_default(30)->as_int();

    this->auto_sleep = THEKERNEL->config->value(power_checksum, auto_sleep_checksum )->by_default(true)->as_bool();
    this->auto_sleep_min = THEKERNEL->config->value(power_checksum, auto_sleep_min_checksum )->by_default(30)->as_number();


    this->enable_light = THEKERNEL->config->value(get_checksum("switch"), get_checksum("light"), get_checksum("startup_state"))->by_default(false)->as_bool();
    this->turn_off_light_min = THEKERNEL->config->value(light_checksum, turn_off_min_checksum )->by_default(10)->as_number();

    this->stop_on_cover_open = THEKERNEL->config->value( stop_on_cover_open_checksum )->by_default(false)->as_bool(); // @deprecated


    this->switch_power_12(1);
    this->switch_power_24(1);

    this->main_button_LED_R.set(0);
    this->main_button_LED_G.set(0);
    this->main_button_LED_B.set(0);

    timer.setFrequency(this->poll_frequency);
    timer.start();

    mbed::InterruptIn *e_stop_interrupt_in = this->e_stop.interrupt_pin();

    e_stop_interrupt_in->rise(this, &MainButton::e_stop_irq);
    e_stop_interrupt_in->fall(this, &MainButton::e_stop_irq);
}

void MainButton::switch_power_12(int state)
{
    this->PS12.set(state);
}

void MainButton::switch_power_24(int state)
{
    this->PS24.set(state);
}

// once a second, from the polling timer: what draws 12V decides when the power fan may stop
void MainButton::check_12v()
{
    struct pad_switch pad;
    bool vacuum_on = SwitchPool::get_state(get_checksum("vacuum"), &pad) && pad.state;
    bool toolsensor_on = SwitchPool::get_state(get_checksum("toolsensor"), &pad) && pad.state;

    using_12v = THEKERNEL->get_laser_mode() || vacuum_on || toolsensor_on;
}

// both edges fire, so check the button rather than halting on release too
void MainButton::e_stop_irq() {
    if(this->e_stop.get()) machine_task.halt(E_STOP, "e-stop");
}

// the power fan runs while anything draws current, and for a while after
void MainButton::update_power(uint8_t state)
{
    if((state == IDLE || state == SLEEP) && !using_12v) {
        if(us_ticker_read() - power_fan_countdown_us > (uint32_t)power_fan_delay_s * 1000000) switch_power_12(0);
    } else if(state != ALARM) {
        switch_power_12(1);
        power_fan_countdown_us = us_ticker_read();
    }
}

// idle for long enough and the machine puts itself to sleep, or at least turns the light off
void MainButton::update_timeouts(uint8_t state)
{
    if(auto_sleep && auto_sleep_min > 0) {
        if(state != IDLE) sleep_countdown_us = us_ticker_read();
        else if(us_ticker_read() - sleep_countdown_us > (uint32_t)auto_sleep_min * 60 * 1000000) go_to_sleep();
    }

    if(enable_light && turn_off_light_min > 0) {
        if(state != IDLE) {
            light_countdown_us = us_ticker_read();
            SwitchPool::set_state(light_checksum, true);
        } else if(us_ticker_read() - light_countdown_us > (uint32_t)turn_off_light_min * 60 * 1000000) {
            SwitchPool::set_state(light_checksum, false);
        }
    }
}

void MainButton::go_to_sleep()
{
    switch_power_12(0);
    switch_power_24(0);
    THEKERNEL->set_sleeping(true);
    machine_task.halt(MANUAL, "stopped by button");
}

void MainButton::short_press(uint8_t state)
{
    switch(state) {
        case IDLE: case RUN: case HOME: machine_task.halt(MANUAL, "stopped by button"); break;
        case HOLD:  machine_task.hold(false); break;
        case SLEEP: system_reset(false); break;
        case ALARM: break;   // it takes a long press to clear an alarm
    }
}

void MainButton::long_press(uint8_t state)
{
    switch(state) {
        case IDLE:
            if(long_press_enable == "Sleep") go_to_sleep();
            break;
        case RUN: case HOME: machine_task.halt(MANUAL, "stopped by button"); break;
        case HOLD:  machine_task.hold(false); break;
        case SLEEP: system_reset(false); break;
        case ALARM:
            // a reason above 20 is a fault the machine cannot simply be unlocked from
            if(machine_task.halt_reason() > 20) {
                system_reset(false);
            } else {
                machine_task.clear_halt();
                printk("UnKill button pressed, Halt cleared\r\n");
            }
            break;
    }
}

// the LED is the only thing telling the operator what the machine thinks it is doing
void MainButton::update_led(uint8_t state)
{
    bool blink = ++hold_toggle % 4 < 2;
    uint8_t r= 0, g= 0, b= 0;
    switch(state) {
        case IDLE:    b= 1; break;
        case RUN:     g= 1; break;
        case HOME:    r= 1; g= 1; break;
        case HOLD:    g= blink; break;
        case ALARM:   r= 1; break;
        case SLEEP:   r= 1; g= 1; b= 1; break;
        case SUSPEND: b= blink; break;
        case WAIT:    r= blink; g= blink; break;
    }
    main_button_LED_R.set(r);
    main_button_LED_G.set(g);
    main_button_LED_B.set(b);
}

void MainButton::handle_button()
{
    bool pressed= button_state == BUTTON_SHORT_PRESSED || button_state == BUTTON_LONG_PRESSED;
    if(!e_stop.get() && button_state != BUTTON_LED_UPDATE && !pressed) return;

    uint8_t state = THEKERNEL->get_state();

    if(stop_on_cover_open && !machine_task.is_halted() && player.is_playing() && !endstops.cover_closed())
        machine_task.halt(COVER_OPEN, "cover open");

    update_power(state);
    update_timeouts(state);

    if(button_state == BUTTON_SHORT_PRESSED) short_press(state);
    else if(button_state == BUTTON_LONG_PRESSED) long_press(state);
    else update_led(state);

    button_state = NONE;
}

// Polls the button and runs everything the button does: the operator side of the machine
// does not wait for the main loop to get round to it.
void MainButton::button_tick()
{
    if(++second_counter >= poll_frequency) {
        second_counter = 0;
        check_12v();
    }

    handle_button();

    // held: remember when, so releasing can tell a short press from a long one
    if (this->main_button.get()) {
        if (!this->button_pressed) {
            this->button_pressed = true;
            this->button_press_time = us_ticker_read();
        }
    } else {
        if (this->button_pressed) {
            if (us_ticker_read() - this->button_press_time > this->long_press_time_ms * 1000) {
                button_state = BUTTON_LONG_PRESSED;
            } else {
                button_state = BUTTON_SHORT_PRESSED;
            }
            this->button_pressed = false;
        } else {
            if(++led_update_timer > this->poll_frequency * 0.2) {
                button_state = BUTTON_LED_UPDATE;
                led_update_timer = 0;
            }
        }
    }
}
