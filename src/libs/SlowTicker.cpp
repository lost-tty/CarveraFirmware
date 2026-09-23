#include "SlowTicker.h"
#include "Kernel.h"

void SlowTicker::on_module_loaded() {
    taskHandle = xTaskGetCurrentTaskHandle();
    this->register_for_event(ON_IDLE);
}

void SlowTicker::start() {
    timer.start();
}

// a SoftTimer callback runs on the timer service task, not in an interrupt, so the plain
// task API is the right one here
void SlowTicker::timerCallback() {
    xTaskNotifyGive(taskHandle);
}

void SlowTicker::on_idle(void*) {
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
        THEKERNEL->call_event(ON_SECOND_TICK);
    }
}
