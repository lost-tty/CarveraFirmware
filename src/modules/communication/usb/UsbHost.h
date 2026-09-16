#ifndef USBHOST_H
#define USBHOST_H

#include "Module.h"
#include "libs/RingBuffer.h"
#include "Pendant.h"

// USB host port with TinyUSB, polled from ON_IDLE. HID handlers queue command lines that are
// dispatched from the main loop like console input.
class UsbHost : public Module {
    public:
        UsbHost() : pendant(*this) {}
        void on_module_loaded();
        void on_idle(void* argument);
        void on_main_loop(void* argument);

        void queue_line(const char* line);
        void on_hid_report(uint8_t dev_addr, uint8_t idx, const uint8_t* report, uint16_t len);
        void on_hid_mount(uint8_t dev_addr, uint8_t idx, bool present);
        void on_hid_protocol(uint8_t idx, uint8_t protocol);

        static UsbHost* instance;

    private:
        bool init_controller();

        Pendant pendant;
        RingBuffer<char, 128> lines;
        bool running = false;
        bool reenumerated = false;
};

#endif
