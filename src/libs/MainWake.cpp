#include "MainWake.h"

static TaskHandle_t main_task = nullptr;

void main_wake_init(TaskHandle_t main)
{
    main_task = main;
}

void wake_main()
{
    if (main_task != nullptr)
        xTaskNotifyGive(main_task);
}

void wake_main_from_isr(BaseType_t *woken)
{
    if (main_task != nullptr)
        vTaskNotifyGiveFromISR(main_task, woken);
}

void wait_main(TickType_t at_most)
{
    ulTaskNotifyTake(pdTRUE, at_most);
}
