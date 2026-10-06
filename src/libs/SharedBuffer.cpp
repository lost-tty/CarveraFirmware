#include "SharedBuffer.h"

#include "FreeRTOS.h"
#include "task.h"

bool SharedBuffer::take(const void* who)
{
    taskENTER_CRITICAL();
    bool ok = holder == nullptr || holder == who;
    if (ok)
        holder = who;

    taskEXIT_CRITICAL();
    return ok;
}

void SharedBuffer::give(const void* who)
{
    taskENTER_CRITICAL();
    if (holder == who)
        holder = nullptr;

    taskEXIT_CRITICAL();
}
