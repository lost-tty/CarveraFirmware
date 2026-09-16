#include "UsbHost.h"
#include "libs/Kernel.h"
#include "libs/Logging.h"
#include "libs/Pin.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "mbed.h"
#include "tusb.h"
#include "host/hcd.h"

#define usb_host_checksum    CHECKSUM("usb_host")
#define enable_checksum      CHECKSUM("enable")
#define enable_pin_checksum  CHECKSUM("enable_pin")

UsbHost* UsbHost::instance = nullptr;


extern "C" void USB_IRQHandler(void) { tuh_int_handler(0, true); }
extern "C" void hcd_int_enable(uint8_t)  { NVIC_EnableIRQ(USB_IRQn); }   // board hooks TinyUSB expects
extern "C" void hcd_int_disable(uint8_t) { NVIC_DisableIRQ(USB_IRQn); }

extern "C" void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const*, uint16_t)
{
    if (UsbHost::instance) UsbHost::instance->on_hid_mount(dev_addr, idx, true);
    tuh_hid_receive_report(dev_addr, idx);
}

extern "C" void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx)
{
    if (UsbHost::instance) UsbHost::instance->on_hid_mount(dev_addr, idx, false);
}

extern "C" void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
    if (UsbHost::instance) UsbHost::instance->on_hid_report(dev_addr, idx, report, len);
    tuh_hid_receive_report(dev_addr, idx);
}

void UsbHost::on_module_loaded()
{
    if (!THEKERNEL->config->value(usb_host_checksum, enable_checksum)->by_default(false)->as_bool()) return;

    Pin enable_pin;
    enable_pin.from_string(THEKERNEL->config->value(usb_host_checksum, enable_pin_checksum)->by_default("1.19!")->as_string())->as_output();
    enable_pin.set(false);

    if (!init_controller()) { printk("USB host: clock not ready, disabled\n"); return; }
    instance = this;
    running = tuh_init(0);
    if (!running) { printk("USB host init failed\n"); return; }
    enable_pin.set(true);
    // VBUS is hard wired on this board: a device present at reset never produces the connect-change
    // interrupt the OHCI driver relies on, so do the initial port scan its hcd_init omits
    if (hcd_port_connect_status(0)) hcd_event_device_attach(0, false);

    register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);
}

// LPC1768 USB block as OHCI host on port 1 (P0.29/P0.30)
bool UsbHost::init_controller()
{
    const uint32_t HOST_CLK_EN = 1 << 0, PORTSEL_CLK_EN = 1 << 3, AHB_CLK_EN = 1 << 4;
    const uint32_t clocks = HOST_CLK_EN | PORTSEL_CLK_EN | AHB_CLK_EN;

    NVIC_DisableIRQ(USB_IRQn);
    LPC_SC->PCONP |= 1UL << 31;
    LPC_USB->USBClkCtrl |= clocks;
    for (int i = 0; (LPC_USB->USBClkSt & clocks) != clocks; i++) {
        if (i > 100000) return false;
    }
    LPC_USB->OTGStCtrl |= 1;                  // port 1 to host controller
    LPC_USB->USBClkCtrl &= ~PORTSEL_CLK_EN;
    LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~((3 << 26) | (3 << 28))) | (1 << 26) | (1 << 28);  // USB_D+/D-

    NVIC_SetVector(USB_IRQn, (uint32_t)USB_IRQHandler);
    NVIC_SetPriority(USB_IRQn, 6);   // tuh_init() enables it
    return true;
}

void UsbHost::on_idle(void*)
{
    tuh_task();
    pendant.tick();
}

void UsbHost::on_hid_protocol(uint8_t idx, uint8_t protocol)
{
    pendant.on_protocol(idx, protocol);
}

void UsbHost::on_hid_mount(uint8_t dev_addr, uint8_t idx, bool present)
{
    if (tuh_hid_interface_protocol(dev_addr, idx) != HID_ITF_PROTOCOL_KEYBOARD) return;
    printk("USB keyboard %s\n", present ? "connected" : "removed");
    if (!present) reenumerated = false;
    pendant.set_device(dev_addr, idx, present);
}

extern "C" void tuh_hid_set_protocol_complete_cb(uint8_t, uint8_t idx, uint8_t protocol)
{
    if (UsbHost::instance) UsbHost::instance->on_hid_protocol(idx, protocol);
}

void UsbHost::on_hid_report(uint8_t dev_addr, uint8_t idx, const uint8_t* report, uint16_t len)
{
    // A keyboard that was already on the bus when the controller started comes out of its first
    // enumeration sending empty reports (seen at cold boot and after reset, never after a replug).
    // A boot keyboard never sends an empty report, so enumerate it once more.
    if (len == 0 && tuh_hid_interface_protocol(dev_addr, idx) == HID_ITF_PROTOCOL_KEYBOARD && !reenumerated) {
        reenumerated = true;
        printk("USB keyboard sends empty reports, re-enumerating\n");
        hcd_event_device_remove(0, false);
        hcd_event_device_attach(0, false);
        return;
    }
    if (tuh_hid_interface_protocol(dev_addr, idx) == HID_ITF_PROTOCOL_KEYBOARD && len >= sizeof(hid_keyboard_report_t)) {
        pendant.on_report(*reinterpret_cast<const hid_keyboard_report_t*>(report));
    }
}

void UsbHost::queue_line(const char* line)
{
    int len = strlen(line);
    if (len + 1 > lines.capacity() - lines.size()) return;
    for (int i = 0; i < len; i++) lines.push_back(line[i]);
    lines.push_back('\n');
}

void UsbHost::on_main_loop(void*)
{
    if (lines.size() == 0) return;
    std::string line;
    char c;
    do {
        lines.pop_front(c);
        if (c != '\n') line += c;
    } while (c != '\n' && lines.size() > 0);

    SerialMessage message{&StreamOutput::NullStream, line, 0};
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
}
