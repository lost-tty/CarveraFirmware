#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#define CFG_TUSB_OS                 OPT_OS_NONE     // polled from ON_IDLE
#define CFG_TUSB_DEBUG              0

#define CFG_TUH_ENABLED             1
#define CFG_TUH_MAX_SPEED           OPT_MODE_FULL_SPEED
#define CFG_TUH_MEM_SECTION         __attribute__((section(".ohci_data")))   // OHCI DMA needs AHB RAM
#define CFG_TUH_MEM_ALIGN           __attribute__((aligned(4)))

#define CFG_TUH_ENUMERATION_BUFSIZE 192
#define CFG_TUH_HUB                 0
#define CFG_TUH_DEVICE_MAX          1
#define CFG_TUH_ENDPOINT_MAX        4
#define CFG_TUH_HID                 2               // a keyboard often exposes two HID interfaces
#define CFG_TUH_HID_EPIN_BUFSIZE    64
#define CFG_TUH_HID_EPOUT_BUFSIZE   64

#endif
