#ifndef USBHOST_H
#define USBHOST_H

#include "Module.h"
#include "libs/RingBuffer.h"
#include "Pendant.h"

#include "FreeRTOS.h"
#include "task.h"

class UsbHost : public Module {
    public:
        UsbHost() : pendant(*this) {}
        void on_module_loaded();
        void on_main_loop(void* argument);

        void queue_line(const char* line);
        void on_hid_report(uint8_t dev_addr, uint8_t idx, const uint8_t* report, uint16_t len);
        void on_hid_mount(uint8_t dev_addr, uint8_t idx, bool present);
        void on_hid_protocol(uint8_t idx, uint8_t protocol);

        static UsbHost* instance;

    private:
        bool init_controller();
        static void run(void *self);

        static const uint32_t k_poll_ms = 50;
        static const uint16_t k_stack_words = 512;
        StackType_t stack[k_stack_words];
        StaticTask_t task;
        TaskHandle_t handle{nullptr};

        Pendant pendant;
        RingBuffer<char, 128> lines;
        bool running = false;
        bool reenumerated = false;
};

extern UsbHost usb_host;


#endif
