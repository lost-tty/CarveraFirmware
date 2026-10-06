#include "MainWake.h"
#include "DeferredWake.h"

static TaskHandle_t main_task = nullptr;

void main_wake_init(TaskHandle_t main)
{
    main_task = main;
    defer_wake_to(WAKE_MAIN, main, 0);
}

void wake_main()
{
    if (main_task != nullptr)
        xTaskNotifyGive(main_task);
}

void wait_main(TickType_t at_most)
{
    ulTaskNotifyTake(pdTRUE, at_most);
}
