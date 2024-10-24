/* Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Provide routines which hook the MRI debug monitor into GCC4MBED projects. */
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <mri.h>
#include <cmsis.h>
#include <malloc.h>
#include "mpu.h"
#include "FreeRTOS.h"
#include "task.h"

unsigned int g_maximumHeapAddress;

static void configureStackSizeLimit(unsigned int stackSizeLimit);
static unsigned int alignTo32Bytes(unsigned int value);
static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress);
static void configureMpuRegionToAccessAllMemoryWithNoCaching(void);

/* Symbols exposed from the linker script. */
extern unsigned int     __bss_start__;
extern unsigned int     __bss_end__;
extern unsigned int     __StackTop;
extern unsigned int     __HeapBase;
extern unsigned int     __AHB_dyn_start;
extern unsigned int     __AHB_end;
extern "C" unsigned int __end__;

HeapRegion_t xHeapRegions[3];

void initHeapRegions() {
    xHeapRegions[0].pucStartAddress = (uint8_t*)&__HeapBase;
    xHeapRegions[0].xSizeInBytes = (uintptr_t)&__StackTop - (uintptr_t)&__HeapBase - __STACK_SIZE - 32;

    xHeapRegions[1].pucStartAddress = (uint8_t*)&__AHB_dyn_start;
    xHeapRegions[1].xSizeInBytes = (uintptr_t)&__AHB_end - (uintptr_t)&__AHB_dyn_start;

    // Terminator entry
    xHeapRegions[2].pucStartAddress = NULL;
    xHeapRegions[2].xSizeInBytes = 0;

    vPortDefineHeapRegions(xHeapRegions);
}

extern "C" int  main(void);
extern "C" void __libc_init_array(void);
extern "C" void exit(int ErrorCode);
extern "C" void _start(void)
{
    int bssSize = (int)&__bss_end__ - (int)&__bss_start__;
    int mainReturnValue;

    memset(&__bss_start__, 0, bssSize);

    if (__STACK_SIZE) {
        configureStackSizeLimit(__STACK_SIZE);
    }

    if (WRITE_BUFFER_DISABLE) {
        disableMPU();
        configureMpuRegionToAccessAllMemoryWithNoCaching();
        enableMPU();
    }

    if (MRI_ENABLE) {
        mriInit(MRI_INIT_PARAMETERS);
        if (MRI_BREAK_ON_INIT)
            __debugbreak();
    }

    initHeapRegions();

    __libc_init_array();
    mainReturnValue = main();
    exit(mainReturnValue);
}

extern "C" void* _sbrk(ptrdiff_t incr) {
    errno = ENOMEM;  // No more memory to allocate
    return (void*)-1;
}

static void configureStackSizeLimit(unsigned int stackSizeLimit)
{
    // Note: 32 bytes are reserved to fall between top of heap and top of stack for minimum MPU guard region.
    g_maximumHeapAddress = alignTo32Bytes((unsigned int)&__StackTop - stackSizeLimit - 32);
    configureMpuToCatchStackOverflowIntoHeap(g_maximumHeapAddress);
}

static unsigned int alignTo32Bytes(unsigned int value)
{
    return (value + 31) & ~31;
}

static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress)
{
#define MPU_REGION_SIZE_OF_32_BYTES ((5-1) << MPU_RASR_SIZE_SHIFT)  // 2^5 = 32 bytes.

    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(maximumHeapAddress);
    setMPURegionAttributeAndSize(MPU_REGION_SIZE_OF_32_BYTES | MPU_RASR_ENABLE);
    enableMPUWithDefaultMemoryMap();
}

static void configureMpuRegionToAccessAllMemoryWithNoCaching(void)
{
    static const uint32_t regionToStartAtAddress0 = 0U;
    static const uint32_t regionReadWrite = 1  << MPU_RASR_AP_SHIFT;
    static const uint32_t regionSizeAt4GB = 31 << MPU_RASR_SIZE_SHIFT; /* 4GB = 2^(31+1) */
    static const uint32_t regionEnable    = MPU_RASR_ENABLE;
    static const uint32_t regionSizeAndAttributes = regionReadWrite | regionSizeAt4GB | regionEnable;
    uint32_t regionIndex = __STACK_SIZE ? getHighestMPUDataRegionIndex() - 1 : getHighestMPUDataRegionIndex();

    prepareToAccessMPURegion(regionIndex);
    setMPURegionAddress(regionToStartAtAddress0);
    setMPURegionAttributeAndSize(regionSizeAndAttributes);
}


extern "C" int __real__read(int file, char *ptr, int len);
extern "C" int __wrap__read(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return mriNewlib_SemihostRead(file, ptr, len);
    return __real__read(file, ptr, len);
}


extern "C" int __real__write(int file, char *ptr, int len);
extern "C" int __wrap__write(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return mriNewlib_SemihostRead(file, ptr, len);
    return __real__write(file, ptr, len);
}


extern "C" int __real__isatty(int file);
extern "C" int __wrap__isatty(int file)
{
    /* Hardcoding the stdin/stdout/stderr handles to be interactive tty devices, unlike mbed.ar */
    if (file < 3)
        return 1;
    return __real__isatty(file);
}


extern "C" int __wrap_semihost_connected(void)
{
    /* MRI makes it look like there is no mbed interface attached since it disables the JTAG portion but MRI does
       support some of the mbed semihost calls when it is running so force it to return -1, indicating that the
       interface is attached. */
    return -1;
}


extern "C" void abort(void)
{
    if (MRI_ENABLE)
        __debugbreak();

    exit(1);
}


extern "C" void __cxa_pure_virtual(void)
{
    abort();
}

extern "C" void* __wrap__malloc_r(struct _reent *r, size_t size)
{
    return pvPortMalloc(size);
}

extern "C" void __wrap__free_r(struct _reent *r, void *ptr)
{
    vPortFree(ptr);
}

