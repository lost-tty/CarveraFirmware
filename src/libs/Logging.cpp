// Logging.cpp
#include "Logging.h"
#include "libs/Kernel.h"

void printk(const char* format, ...) {
    va_list args;
    va_start(args, format);

    THEKERNEL->streams.vprintf(format, args);
    
    va_end(args);
}
