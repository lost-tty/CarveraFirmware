#include "Pendant.h"
#include "UsbHost.h"
#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "utils.h"
#include "SpindlePublicAccess.h"
#include "SpindleControl.h"
#include "SwitchPublicAccess.h"
#include "SwitchPool.h"
#include "checksumm.h"
#include "mbed.h"
#include "tusb.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include "modules/robot/MachineTask.h"

#define switch_checksum CHECKSUM("switch")
#define state_checksum  CHECKSUM("state")

static const float step_sizes[]  = { 0.01f, 0.1f, 1.0f, 10.0f };
static const float cont_scale[]  = { 0.1f, 0.5f };           // of the axis max rate
static const uint32_t SEGMENT_US = 100000;                   // continuous jog feeds 100 ms of motion at a time

// numpad and full keyboard entries side by side; "shifted" = NumLock held or Shift
const Pendant::Key Pendant::keys[] = {
    { HID_KEY_KEYPAD_4,        false, JOG,     'X', -1 }, { HID_KEY_ARROW_LEFT,  false, JOG, 'X', -1 },
    { HID_KEY_KEYPAD_6,        false, JOG,     'X', +1 }, { HID_KEY_ARROW_RIGHT, false, JOG, 'X', +1 },
    { HID_KEY_KEYPAD_2,        false, JOG,     'Y', -1 }, { HID_KEY_ARROW_DOWN,  false, JOG, 'Y', -1 },
    { HID_KEY_KEYPAD_8,        false, JOG,     'Y', +1 }, { HID_KEY_ARROW_UP,    false, JOG, 'Y', +1 },
    { HID_KEY_KEYPAD_3,        false, JOG,     'Z', -1 }, { HID_KEY_PAGE_DOWN,   false, JOG, 'Z', -1 },
    { HID_KEY_KEYPAD_9,        false, JOG,     'Z', +1 }, { HID_KEY_PAGE_UP,     false, JOG, 'Z', +1 },
    { HID_KEY_KEYPAD_1,        false, JOG,     'A', -1 },
    { HID_KEY_KEYPAD_7,        false, JOG,     'A', +1 },
    { HID_KEY_KEYPAD_4,        true,  ZERO,    'X',  0 }, { HID_KEY_ARROW_LEFT,  true,  ZERO, 'X', 0 },
    { HID_KEY_KEYPAD_6,        true,  ZERO,    'X',  0 }, { HID_KEY_ARROW_RIGHT, true,  ZERO, 'X', 0 },
    { HID_KEY_KEYPAD_2,        true,  ZERO,    'Y',  0 }, { HID_KEY_ARROW_DOWN,  true,  ZERO, 'Y', 0 },
    { HID_KEY_KEYPAD_8,        true,  ZERO,    'Y',  0 }, { HID_KEY_ARROW_UP,    true,  ZERO, 'Y', 0 },
    { HID_KEY_KEYPAD_3,        true,  ZERO,    'Z',  0 }, { HID_KEY_PAGE_DOWN,   true,  ZERO, 'Z', 0 },
    { HID_KEY_KEYPAD_9,        true,  ZERO,    'Z',  0 }, { HID_KEY_PAGE_UP,     true,  ZERO, 'Z', 0 },
    { HID_KEY_KEYPAD_ADD,      false, MODE,     0,  +1 }, { HID_KEY_EQUAL,       false, MODE,  0, +1 },
    { HID_KEY_KEYPAD_SUBTRACT, false, MODE,     0,  -1 }, { HID_KEY_MINUS,       false, MODE,  0, -1 },
    { HID_KEY_KEYPAD_ADD,      true,  FEED,     0,  +1 }, { HID_KEY_EQUAL,       true,  FEED,  0, +1 },
    { HID_KEY_KEYPAD_SUBTRACT, true,  FEED,     0,  -1 }, { HID_KEY_MINUS,       true,  FEED,  0, -1 },
    { HID_KEY_KEYPAD_5,        false, HOLD,     0,   0 }, { HID_KEY_SPACE,       false, HOLD,  0,  0 },
    { HID_KEY_KEYPAD_0,        true,  HOME,     0,   0 },
    { HID_KEY_KEYPAD_MULTIPLY, true,  SPINDLE,  0,   0 },
    { HID_KEY_KEYPAD_DIVIDE,   false, VACUUM,   0,   0 },
    { HID_KEY_KEYPAD_DIVIDE,   true,  LIGHT,    0,   0 },
    { HID_KEY_KEYPAD_ENTER,    false, RESUME,   0,   0 },
    { HID_KEY_BACKSPACE,       false, ABORT,    0,   0 }, { HID_KEY_ESCAPE,      false, ABORT, 0,  0 },
    { HID_KEY_BACKSPACE,       true,  UNLOCK,   0,   0 }, { HID_KEY_ESCAPE,      true,  UNLOCK, 0, 0 },
};

static bool in_report(const hid_keyboard_report_t& r, uint8_t key)
{
    for (uint8_t k : r.keycode) if (k == key) return true;
    return false;
}

void Pendant::set_device(uint8_t addr, uint8_t idx, bool is_present)
{
    dev_addr = addr;
    dev_idx = idx;
    present = is_present;
    boot_protocol = false;
    protocol_tries = 0;
    next_protocol_try = us_ticker_read();
    leds = 0xFF;
    held_axis = 0;
}

// TinyUSB assumes boot protocol after its own request even if the keyboard stalled it (still
// starting up at machine power-on), so ask again until the keyboard confirms
void Pendant::on_protocol(uint8_t idx, uint8_t protocol)
{
    if (!present || idx != dev_idx) return;
    boot_protocol = (protocol == HID_PROTOCOL_BOOT);
    if (boot_protocol) printk("USB keyboard ready\n");
}

void Pendant::on_report(const hid_keyboard_report_t& report)
{
    if (!boot_protocol) return;   // report protocol layout differs
    bool shifted = (report.modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT)) || in_report(report, HID_KEY_NUM_LOCK);
    for (uint8_t k : prev.keycode) if (k != 0 && !in_report(report, k)) key_up(k);
    for (uint8_t k : report.keycode) if (k != 0 && !in_report(prev, k)) key_down(k, shifted);
    prev = report;
}

void Pendant::line(const char* fmt, ...)
{
    char buf[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    host.queue_line(buf);
}

void Pendant::key_down(uint8_t key, bool shifted)
{
    for (auto& k : keys) {
        if (k.key != key || k.shifted != shifted) continue;
        if (machine_task.is_halted() && k.action != UNLOCK && k.action != HOME) return;
        switch (k.action) {
            case JOG:
                held_axis = k.axis;
                held_dir = k.dir;
                jog(k.axis, k.dir);
                break;
            case MODE: {
                const int n = 4 + 2;
                mode = (mode + n + k.dir) % n;
                if (mode < 4) printk("Jog step %.2f mm\n", step_sizes[mode]);
                else printk("Jog continuous %d%%\n", (int)(cont_scale[mode - 4] * 100));
                break;
            }
            case FEED:
                feed_pct = feed_pct + 10 * k.dir;
                if (feed_pct < 10) feed_pct = 10;
                if (feed_pct > 200) feed_pct = 200;
                line("M220 S%d", feed_pct);
                break;
            case HOLD:
                if (THEKERNEL->is_feed_hold_enabled()) THEKERNEL->set_feed_hold(!THEKERNEL->get_feed_hold());
                else line(THEKERNEL->is_suspending() ? "resume" : "suspend");
                break;
            case ABORT:
                machine_task.halt(MANUAL, "stopped from pendant");
                break;
            case UNLOCK:  line("$X"); break;
            case HOME:    line("$H"); break;
            case RESUME:  line("resume"); break;
            case ZERO:    line("G10 L20 P0 %c0", k.axis); break;
            case VACUUM:  toggle_switch("vacuum"); break;
            case LIGHT:   toggle_switch("light"); break;
            case SPINDLE: {
                struct spindle_status ss;
                if (spindle_control == nullptr) break;
                spindle_control->get_status(&ss);
                if (ss.state) line("M5");
                else if (ss.target_rpm > 0) line("M3 S%d", (int)ss.target_rpm);
                break;
            }
        }
        return;
    }
}

void Pendant::key_up(uint8_t key)
{
    for (auto& k : keys) {
        if (k.key == key && k.action == JOG && k.axis == held_axis) held_axis = 0;
    }
}

void Pendant::jog(char axis, int8_t dir)
{
    float distance;
    if (mode < 4) {
        distance = step_sizes[mode];
        held_axis = 0;
    } else {
        int i = axis >= 'X' ? axis - 'X' : axis - 'A' + 3;
        if (i >= THEROBOT.get_number_registered_motors()) return;
        distance = THEROBOT.motor_max_rate(i) * cont_scale[mode - 4] * SEGMENT_US / 1e6f;
        last_segment = us_ticker_read();
    }
    if (mode < 4) line("$J %c%.3f", axis, dir * distance);
    else line("$J %c%.3f F%.2f", axis, dir * distance, cont_scale[mode - 4]);
}

void Pendant::toggle_switch(const char* name)
{
    struct pad_switch pad;
    if (!SwitchPool::get_state(get_checksum(name), &pad)) return;
    bool on = !pad.state;
    SwitchPool::set_state(get_checksum(name), on);
}

void Pendant::tick()
{
    uint32_t now = us_ticker_read();
    if (present && !boot_protocol && protocol_tries < 20 && (int32_t)(now - next_protocol_try) >= 0) {
        protocol_tries++;
        next_protocol_try = now + 500000;
        tuh_hid_set_protocol(dev_addr, dev_idx, HID_PROTOCOL_BOOT);
        if (protocol_tries == 20) printk("USB keyboard did not accept boot protocol\n");
    }
    if (held_axis && mode >= 4 && now - last_segment >= SEGMENT_US) jog(held_axis, held_dir);
    if (now - last_led_check >= 50000) {
        last_led_check = now;
        update_leds(now);
    }
}

// NumLock LED (the only one on a numpad): off idle, on running, slow blink hold, fast blink alarm,
// short blink not homed. CapsLock: continuous jog mode. ScrollLock: alarm.
void Pendant::update_leds(uint32_t now)
{
    if (!present) return;
    bool fast = (now / 250000) & 1, slow = (now / 1000000) & 1, pulse = (now % 1000000) < 100000;
    uint8_t state = THEKERNEL->get_state();
    uint8_t v = 0;
    if (state == ALARM)                       v |= fast ? KEYBOARD_LED_NUMLOCK : 0;
    else if (state == HOLD || state == SUSPEND) v |= slow ? KEYBOARD_LED_NUMLOCK : 0;
    else if (state == RUN || state == HOME)   v |= KEYBOARD_LED_NUMLOCK;
    else if (!THEROBOT.is_homed_all_axes())   v |= pulse ? KEYBOARD_LED_NUMLOCK : 0;
    if (mode >= 4) v |= KEYBOARD_LED_CAPSLOCK;
    if (state == ALARM && fast) v |= KEYBOARD_LED_SCROLLLOCK;
    if (v == leds) return;
    static uint8_t out;
    out = v;
    if (tuh_hid_set_report(dev_addr, dev_idx, 0, HID_REPORT_TYPE_OUTPUT, &out, 1)) leds = v;   // else retry next tick
}
