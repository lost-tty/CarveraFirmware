#pragma once

#include "FreeRTOS.h"
#include "task.h"

void main_wake_init(TaskHandle_t main);
void wake_main();
void wake_main_from_isr(BaseType_t *woken);
void wait_main(TickType_t at_most);
