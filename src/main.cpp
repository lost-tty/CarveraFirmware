/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/ToolHead.h"
#include "modules/tools/spindle/SpindleMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/zprobe/ZProbe.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/tools/atc/ATCHandler.h"
#include "modules/utils/script/Scripts.h"
#include "modules/utils/wifi/WifiProvider.h"
#include "modules/utils/webserver/WebServer.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/MachineTask.h"
#include "modules/robot/Robot.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/mainbutton/MainButton.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/communication/Source.h"
#include "modules/communication/WirelessProbe.h"
#include "modules/communication/usb/UsbHost.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Robot.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <mri.h>

// #include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

// Debug
#include "libs/SerialMessage.h"

//#include "libs/USBDevice/SDCard/SDCard.h"
#include "libs/USBDevice/SDCard/SDFileSystem.h"
// #include "libs/USBDevice/USBSerial/USBSerial.h"
// #include "libs/USBDevice/DFU.h"
#include "libs/SDFAT.h"
#include "Logging.h"

#include "libs/Watchdog.h"

#include "version.h"

// disable MSD
#define DISABLEMSD
#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
#define watchdog_timeout_checksum  CHECKSUM("watchdog_timeout")

extern "C" void vPortSVCHandler(void);
extern "C" void xPortPendSVHandler(void);
extern "C" void xPortSysTickHandler(void);

#if (configSUPPORT_STATIC_ALLOCATION == 1)

// Static allocation for the Idle Task
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

// Static allocation for the Timer Task
#if (configUSE_TIMERS == 1)
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
#endif

extern "C" {

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize ) {
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = xIdleStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize ) {
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = xTimerStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

#endif // configUSE_TIMERS

} // extern "C"

#endif // configSUPPORT_STATIC_ALLOCATION

SDFileSystem sd (P0_18, P0_17, P0_15, P0_16, 12000000);

SDFAT mounter ("sd", &sd);

GPIO leds[4] = {
    GPIO(P4_29),
    GPIO(P4_28),
	GPIO(P0_4),
    GPIO(P1_17)
};

Watchdog watchdog (10000, WDT_RESET);  // 10 seconds default, WDT_RESET
Kernel kernel;
Conveyor THECONVEYOR __attribute__((section("AHBSRAM")));
Robot THEROBOT;
GcodeDispatch gcode_dispatch;
SourceStack sources;
SimpleShell simpleshell __attribute__((section("AHBSRAM")));
WifiProvider wifi_provider;
WebServer web_server (&wifi_provider);

Player player;
WirelessProbe wireless_probe;
MainButton mainbutton;
ATCHandler atc_handler;
Scripts scripts;
Endstops endstops;
Laser laser;
// the pools keep the M code slots they register, so they outlive the registry
SwitchPool switch_pool;
TemperatureControlPool temperature_control_pool;
ZProbe zprobe;
UsbHost usb_host;

Kernel* THEKERNEL = &kernel;

serial_t console;

void init() {
    // Default pins to low status
    for (int i = 0; i < 4; i++){
        leds[i].output();
        leds[i]= 0;
    }

    THEKERNEL->init();

    THECONVEYOR.init();
    THEKERNEL->add_module(&THECONVEYOR);

    gcode_dispatch.init();
    THEKERNEL->add_module(&gcode_dispatch);
    THEKERNEL->add_module(&sources); // before player: on halt a script pops its state before the suspend state

    THEROBOT.init();
    THEKERNEL->add_module(&THEROBOT);

    THEKERNEL->add_module(&simpleshell);

    printk("Smoothie Running @%luMHz\r\n", (unsigned long)(SystemCoreClock / 1000000));
    simpleshell.version_command("", &THEKERNEL->streams);

    bool sdok = (sd.disk_initialize() == 0);
    if(!sdok) printk("SDCard failed to initialize\r\n");

    #ifdef NONETWORK
        printk("NETWORK is disabled\r\n");
    #endif

    // Create and add main modules
    THEKERNEL->add_module(&player);
    THEKERNEL->add_module(&atc_handler);
    THEKERNEL->add_module(&scripts);
    THEKERNEL->add_module(&wireless_probe);
    THEKERNEL->add_module(&usb_host);
    THEKERNEL->add_module(&mainbutton);
    THEKERNEL->add_module(&wifi_provider);
    THEKERNEL->add_module(&web_server);

    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    #ifndef NO_TOOLS_SWITCH
    switch_pool.load_tools();
    #endif

    // #ifndef NO_TOOLS_TEMPERATURECONTROL
    // Note order is important here must be after extruder so Tn as a parameter will get executed first
    temperature_control_pool.load_tools();

    // #endif
    #ifndef NO_TOOLS_ENDSTOPS
    THEKERNEL->add_module(&endstops);
    #endif
    #ifndef NO_TOOLS_LASER
    THEKERNEL->add_module(&laser);
    #endif
    THEKERNEL->add_module(&tool_head);

    #ifndef NO_TOOLS_SPINDLE
    SpindleMaker *sm = new SpindleMaker();
    sm->load_spindle();
    delete sm;
    #endif
    #ifndef NO_TOOLS_ZPROBE
    THEKERNEL->add_module(&zprobe);
    #endif

    // 10 second watchdog timeout (or config as seconds)
    float t= THEKERNEL->config->value( watchdog_timeout_checksum )->by_default(10.0F)->as_number();
    if (t > 0.1F) {
        watchdog.configure(t * 1000000, WDT_RESET);
        watchdog.arm();
        // NOTE setting WDT_RESET with the current bootloader would leave it in DFU mode which would be suboptimal
        THEKERNEL->add_module(&watchdog);
        printk("Watchdog enabled for %1.3f seconds\n", t);
    } else {
        printk("WARNING Watchdog is disabled\n");
    }

    // clear up the config cache to save some memory
    THEKERNEL->config->config_cache_clear();

    if(THEKERNEL->is_using_leds()) {
        // set some leds to indicate status... led0 init done, led1 mainloop running, led2 idle loop running, led3 sdcard ok
        leds[0]= 1; // indicate we are done with init
        leds[3]= sdok?1:0; // 4th led indicates sdcard is available (TODO maye should indicate config was found)
    }

    // start the timers and interrupts
    THECONVEYOR.start(THEROBOT.get_number_registered_motors());
    
    THEKERNEL->step_ticker.start();
    THEKERNEL->slow_ticker.start();
    machine_task.start();
}

// no printf: this can run on the 512 byte main stack with interrupts off, and vfprintf
// needs more than that. the file and line are in the registers for the debugger to read
extern "C" void vAssertCalled(const char *file, int line) {
    volatile const char *f= file;
    volatile int l= line;
    (void)f; (void)l;
    __debugbreak();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    /* Log or print the overflow information */
    printk("Stack overflow detected in task: %s\n", pcTaskName);

    /* Trigger MRI abort */
    abort();
}

void vTaskMainLoop(void *pvParameters) {
    uint16_t cnt = 0;

    init();

    machine_task.post_startup();

    printk("Mainloop started\n");

    while (true) {
        if(THEKERNEL->is_using_leds()) {
            // flash led 2 to show we are alive
            leds[1]= (cnt++ & 0x1000) ? 1 : 0;
        }

        THEKERNEL->serve_main();
        vTaskDelay(1);
    }
}

#ifndef MAINLOOP_STACK_SIZE
#define MAINLOOP_STACK_SIZE 1024
#endif

StackType_t mainLoopStackBuffer[MAINLOOP_STACK_SIZE];
StaticTask_t mainLoopTaskBuffer;

int main() {
    serial_init(&console, P2_8, P2_9);
    serial_baud(&console, DEFAULT_SERIAL_BAUD_RATE);

    NVIC_SetVector(SVCall_IRQn, (uintptr_t)vPortSVCHandler);
    NVIC_SetVector(PendSV_IRQn, (uintptr_t)xPortPendSVHandler);
    NVIC_SetVector(SysTick_IRQn, (uintptr_t)xPortSysTickHandler);

    // Create a FreeRTOS task main loop
    TaskHandle_t xHandle = xTaskCreateStatic(
        vTaskMainLoop,        // Task function
        "MainLoop",           // Task name (for debugging)
        MAINLOOP_STACK_SIZE,  // Stack size (in words, not bytes)
        NULL,                 // Task parameters (none)
        1,                    // Task priority
        mainLoopStackBuffer,
        &mainLoopTaskBuffer
    );

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The program should never reach this point
    // This is only reached if there is insufficient memory to start the scheduler
    abort();

    return 0;
}
