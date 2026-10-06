/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "DeferredWake.h"

volatile bool deferred_due[WAKE_WAITERS];

namespace {
    struct Waiter {
        TaskHandle_t task;
        UBaseType_t index;
    };
    Waiter waiters[WAKE_WAITERS];
}

void defer_wake_to(DeferredWaiter who, TaskHandle_t task, UBaseType_t index)
{
    waiters[who].index = index;
    waiters[who].task = task;
}

// A flag set again after it was read here pends the RIT again, so no wake is lost.
extern "C" void RIT_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    for (int i = 0; i < WAKE_WAITERS; i++) {
        if (!deferred_due[i])
            continue;

        deferred_due[i] = false;
        if (waiters[i].task != nullptr)
            vTaskNotifyGiveIndexedFromISR(waiters[i].task, waiters[i].index, &woken);
    }
    portYIELD_FROM_ISR(woken);
}
