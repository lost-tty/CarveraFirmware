#include "MachineTask.h"

#include "Conveyor.h"
#include "Robot.h"
#include "Scripts.h"
#include "libs/Kernel.h"
#include "libs/Watchdog.h"
#include "libs/Logging.h"
#include "mri.h"

MachineTask machine_task;

void MachineTask::start()
{
    xTaskCreateStatic(run, "Machine", k_stack_words, this, k_priority, stack, &task);
}

OnMachine MachineTask::proof() const
{
    if(!on_task()) __debugbreak();
    return OnMachine{};
}

// this task preempts its creator, so it runs before xTaskCreateStatic returns the handle
void MachineTask::run(void *self)
{
    MachineTask *me= (MachineTask *)self;
    me->handle= xTaskGetCurrentTaskHandle();
    THECONVEYOR.wake_on_block(me->handle);
    me->loop();
}

MachineTask::Ref MachineTask::post(Job job, const Gcode &gcode)
{
    if(handle == nullptr || xTaskGetCurrentTaskHandle() == handle) return k_no_ticket;

    while(count >= k_max_tickets) {
        if(THEKERNEL->is_halted()) return k_no_ticket;
        wait_for_room();
    }

    ring[head].kind= Ticket::LINE;
    ring[head].job= job;
    ring[head].gcode= gcode;

    Ref ticket;
    taskENTER_CRITICAL();
    if(THEKERNEL->is_halted()) {
        taskEXIT_CRITICAL();
        return k_no_ticket;
    }
    ticket= ring[head].ticket= ++posted;
    head= (head + 1) % k_max_tickets;
    count++;
    taskEXIT_CRITICAL();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
    return ticket;
}

bool MachineTask::post_jog(const float delta[], uint8_t naxis, float scale)
{
    if(handle == nullptr || xTaskGetCurrentTaskHandle() == handle) return false;
    if(naxis > k_max_actuators || THEKERNEL->is_halted()) return false;
    if(count >= k_max_tickets) return false;

    ring[head].kind= Ticket::JOG;
    for (uint8_t i = 0; i < naxis; ++i) ring[head].move.delta[i]= delta[i];
    ring[head].move.naxis= naxis;
    ring[head].move.scale= scale;

    taskENTER_CRITICAL();
    ring[head].ticket= ++posted;
    head= (head + 1) % k_max_tickets;
    count++;
    taskEXIT_CRITICAL();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
    return true;
}

// the boot script may read positions, which only mean anything after homing
static void startup()
{
    THEROBOT.home_on_startup();
    scripts.boot();
}

void MachineTask::post_startup()
{
    if(post([](Gcode &, OnMachine) { startup(); }, Gcode{}) == k_no_ticket)
        printk("ERROR: no machine task, the machine will not move\n");
}

bool MachineTask::post_stop()
{
    Ref t= post([](Gcode &, OnMachine) { THECONVEYOR.stop_soon(); }, Gcode{});
    if(t == k_no_ticket) return false;
    wait_for(t);
    return !THEKERNEL->is_halted();
}

bool MachineTask::post_drain()
{
    if(on_task()) return THECONVEYOR.wait_for_idle();

    Ref t= post([](Gcode &, OnMachine) { THECONVEYOR.wait_for_idle(); }, Gcode{});
    if(t == k_no_ticket) return false;
    wait_for(t);
    return !THEKERNEL->is_halted();
}

void MachineTask::wait_for(Ref ticket)
{
    while(!done(ticket) && !THEKERNEL->is_halted()) wait_for_room();
}

// serving the main loop from here would post the next job line too, recursing without bound
void MachineTask::wait_for_room()
{
    waiter= xTaskGetCurrentTaskHandle();
    watchdog.alive();
    ulTaskNotifyTakeIndexed(k_room_notify_index, pdTRUE, pdMS_TO_TICKS(k_room_wait_ms));
    waiter= nullptr;
}

void MachineTask::wake_waiter()
{
    TaskHandle_t t= waiter;
    if(t != nullptr) xTaskNotifyGiveIndexed(t, k_room_notify_index);
}

void MachineTask::drop_all()
{
    taskENTER_CRITICAL();
    tail= head;
    count= 0;
    served= posted;
    taskEXIT_CRITICAL();

    wake_waiter();
}

void MachineTask::serve_tickets()
{
    while(count > 0) {
        if(THEKERNEL->is_halted()) {
            drop_all();
            return;
        }

        Ticket t= ring[tail];

        taskENTER_CRITICAL();
        tail= (tail + 1) % k_max_tickets;
        count--;
        taskEXIT_CRITICAL();

        if(t.kind == Ticket::JOG) {
            THEROBOT.jog_move(t.move.delta, t.move.naxis, t.move.scale);
        } else {
            t.job(t.gcode, OnMachine{});
        }

        // the job may have dropped the ring, which already advanced served past this
        if(t.ticket > served) served= t.ticket;
        wake_waiter();
    }
}

// the queue pre-load runs off a clock, so the wait has to come back even when nothing wakes it
void MachineTask::tick()
{
    if(!on_task()) __debugbreak();
    THECONVEYOR.service();
    ulTaskNotifyTakeIndexed(k_notify_index, pdTRUE, pdMS_TO_TICKS(k_poll_ms));
}

void MachineTask::clear_halt()
{
    clearing= true;
    if(handle != nullptr) xTaskNotifyGiveIndexed(handle, k_notify_index);
}

void MachineTask::finish_clear()
{
    clearing= false;
    drop_all();
    THECONVEYOR.flush_queue();

    while(THECONVEYOR.flushing()) tick();

    THEROBOT.reset_position_from_current_actuator_position();
    THEROBOT.set_keepout(true);
}

void MachineTask::loop()
{
    while(true) {
        if(clearing) finish_clear();
        serve_tickets();
        tick();
    }
}
