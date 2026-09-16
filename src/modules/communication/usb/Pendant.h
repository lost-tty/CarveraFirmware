#ifndef PENDANT_H
#define PENDANT_H

#include <cstdint>
#include "class/hid/hid.h"

class UsbHost;

// USB keyboard or numpad as jog pendant. Key table in Pendant.cpp.
class Pendant {
    public:
        Pendant(UsbHost& host) : host(host) {}
        void on_report(const hid_keyboard_report_t& report);
        void set_device(uint8_t dev_addr, uint8_t idx, bool present);
        void on_protocol(uint8_t idx, uint8_t protocol);
        void tick();
        bool has_device() const { return present && boot_protocol; }

    private:
        enum Action : uint8_t { JOG, MODE, HOLD, ABORT, ZERO, HOME, SPINDLE, VACUUM, LIGHT, UNLOCK, RESUME, FEED };
        struct Key { uint8_t key; bool shifted; Action action; char axis; int8_t dir; };
        static const Key keys[];

        void key_down(uint8_t key, bool shifted);
        void key_up(uint8_t key);
        void jog(char axis, int8_t dir);
        void toggle_switch(const char* name);
        void update_leds(uint32_t now);
        void line(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

        UsbHost& host;
        hid_keyboard_report_t prev = {};
        uint8_t mode = 2;               // 0-3 step sizes, 4-5 continuous slow/fast
        int feed_pct = 100;
        char held_axis = 0;
        int8_t held_dir = 0;
        uint32_t last_segment = 0;

        bool present = false;
        bool boot_protocol = false;
        uint8_t protocol_tries = 0;
        uint32_t next_protocol_try = 0;
        uint8_t dev_addr = 0, dev_idx = 0;
        uint8_t leds = 0xFF;
        uint32_t last_led_check = 0;
};

#endif
