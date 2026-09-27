#include "libs/Kernel.h"
#include "MainButton.h"
#include "ConfigTable.h"
#include "Logging.h"
#include "Endstops.h"
#include "Player.h"
#include "SwitchPublicAccess.h"
#include "SwitchPool.h"
#include "SwitchConfig.h"
#include "modules/robot/MachineTask.h"

using namespace std;

static const char *const long_press_actions[] = { "None", "Sleep", "Repeat", nullptr };
enum : uint8_t { LONG_PRESS_NONE, LONG_PRESS_SLEEP, LONG_PRESS_REPEAT };

#define MAIN_BUTTON_CONFIG(X) \
    X(bool,  enable,             "main_button_enable",              true) \
    X(pin,   pin,                "main_button_pin",                 "1.16^") \
    X(pin,   led_r_pin,          "main_button_LED_R_pin",           "1.10") \
    X(pin,   led_g_pin,          "main_button_LED_G_pin",           "1.15") \
    X(pin,   led_b_pin,          "main_button_LED_B_pin",           "1.14") \
    X(int,   poll_frequency,     "main_button_poll_frequency",      20) \
    X(int,   long_press_time_ms, "main_button_long_press_time",     3000) \
    X(enum,  long_press_enable,  "main_button_long_press_enable",   "None", long_press_actions) \
    X(pin,   e_stop_pin,         "e_stop_pin",                      "0.26!^") \
    X(pin,   ps12_pin,           "ps12_pin",                        "0.22") \
    X(pin,   ps24_pin,           "ps24_pin",                        "0.10") \
    X(int,   power_fan_delay_s,  "power_fan_delay_s",               30) \
    X(bool,  stop_on_cover_open, "stop_on_cover_open",              false)
CONFIG_STRUCT(MainButtonConfig, MAIN_BUTTON_CONFIG);
CONFIG_KEYS(main_button_config_keys, MainButtonConfig, MAIN_BUTTON_CONFIG);

#define POWER_CONFIG(X) \
    X(bool,  auto_sleep,     "auto_sleep",     false) \
    X(int,   auto_sleep_min, "auto_sleep_min", 5)
CONFIG_STRUCT(PowerConfig, POWER_CONFIG);
CONFIG_KEYS(power_config_keys, PowerConfig, POWER_CONFIG);

#define LIGHT_CONFIG(X) \
    X(int,   turn_off_min, "turn_off_min", 0)
CONFIG_STRUCT(LightConfig, LIGHT_CONFIG);
CONFIG_KEYS(light_config_keys, LightConfig, LIGHT_CONFIG);
extern MainButton mainbutton;
static void main_button_config_changed(const ConfigTable::Group *, const void *)
{
    mainbutton.configure();
}
CONFIG_GROUPS(main_button_config_groups,
    CFG_GROUP("", main_button_config_keys, MainButtonConfig, main_button_config_changed),
    CFG_GROUP("power", power_config_keys, PowerConfig, main_button_config_changed),
    CFG_GROUP("light", light_config_keys, LightConfig, main_button_config_changed));

// All three groups are built together, so the hook rereads each of them.
void MainButton::configure()
{
    const MainButtonConfig &b = ConfigTable::config<MainButtonConfig>(main_button_config_groups);
    const PowerConfig &p = ConfigTable::config<PowerConfig>(&main_button_config_groups[1]);
    const LightConfig &l = ConfigTable::config<LightConfig>(&main_button_config_groups[2]);
    cfg.poll_frequency = b.poll_frequency;
    cfg.long_press_time_ms = b.long_press_time_ms;
    cfg.long_press_enable = b.long_press_enable;
    cfg.power_fan_delay_s = b.power_fan_delay_s;
    cfg.stop_on_cover_open = b.stop_on_cover_open;
    cfg.auto_sleep = p.auto_sleep;
    cfg.auto_sleep_min = p.auto_sleep_min;
    cfg.turn_off_min = l.turn_off_min;
}


void MainButton::on_module_loaded()
{
    this->using_12v = false;
    this->led_update_timer = 0;
    this->second_counter = 0;
    this->hold_toggle = 0;
    this->button_state = NONE;
    this->button_pressed = false;
    this->sleep_countdown_us = us_ticker_read();
    this->light_countdown_us = us_ticker_read();
    this->power_fan_countdown_us = us_ticker_read();

    const MainButtonConfig &main_button_config =
        ConfigTable::config<MainButtonConfig>(main_button_config_groups);
    configure();
    // SwitchPool's config exists only at boot, so a change requires a restart.
    cfg.light_startup = switch_light_config().startup_state;
    if (!main_button_config.enable) { // @deprecated
        return;
    }

    this->main_button.from_spec(main_button_config.pin)->as_input();
    this->main_button_LED_R.from_spec(main_button_config.led_r_pin)->as_output();
    this->main_button_LED_G.from_spec(main_button_config.led_g_pin)->as_output();
    this->main_button_LED_B.from_spec(main_button_config.led_b_pin)->as_output();

    this->e_stop.from_spec(main_button_config.e_stop_pin)->as_input();
    this->PS12.from_spec(main_button_config.ps12_pin)->as_output();
    this->PS24.from_spec(main_button_config.ps24_pin)->as_output();

    this->switch_power_12(1);
    this->switch_power_24(1);

    this->main_button_LED_R.set(0);
    this->main_button_LED_G.set(0);
    this->main_button_LED_B.set(0);

    timer.setFrequency(cfg.poll_frequency);
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
        if(us_ticker_read() - power_fan_countdown_us > (uint32_t)cfg.power_fan_delay_s * 1000000) switch_power_12(0);
    } else if(state != ALARM) {
        switch_power_12(1);
        power_fan_countdown_us = us_ticker_read();
    }
}

// idle for long enough and the machine puts itself to sleep, or at least turns the light off
void MainButton::update_timeouts(uint8_t state)
{
    if(cfg.auto_sleep && cfg.auto_sleep_min > 0) {
        if(state != IDLE) sleep_countdown_us = us_ticker_read();
        else if(us_ticker_read() - sleep_countdown_us > (uint32_t)cfg.auto_sleep_min * 60 * 1000000) go_to_sleep();
    }

    if(cfg.light_startup && cfg.turn_off_min > 0) {
        if(state != IDLE) {
            light_countdown_us = us_ticker_read();
            SwitchPool::set_state(light_checksum, true);
        } else if(us_ticker_read() - light_countdown_us > (uint32_t)cfg.turn_off_min * 60 * 1000000) {
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
            if(cfg.long_press_enable == LONG_PRESS_SLEEP) go_to_sleep();
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

    if(cfg.stop_on_cover_open && !machine_task.is_halted() && player.is_playing() && !endstops.cover_closed())
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
    if(++second_counter >= (uint32_t)cfg.poll_frequency) {
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
            if (us_ticker_read() - this->button_press_time > (uint32_t)cfg.long_press_time_ms * 1000) {
                button_state = BUTTON_LONG_PRESSED;
            } else {
                button_state = BUTTON_SHORT_PRESSED;
            }
            this->button_pressed = false;
        } else {
            if(++led_update_timer > cfg.poll_frequency * 0.2) {
                button_state = BUTTON_LED_UPDATE;
                led_update_timer = 0;
            }
        }
    }
}
