#include "MachineTask.h"
#include "us_ticker_api.h"
#include "libs/Profile.h"

Profile::Slot Profile::slots[Profile::k_max];
uint8_t Profile::used= 0;

#include "Conveyor.h"
#include "Robot.h"
#include "Scripts.h"
#include "libs/Kernel.h"
#include "libs/StepTicker.h"
#include "libs/Watchdog.h"
#include "libs/Killable.h"
#include "StreamOutput.h"

#include <cstring>
#include "libs/Logging.h"
#include "mri.h"

MachineTask machine_task;

void MachineTask::start()
{
    free_slots= xQueueCreateStatic(k_max_tickets, sizeof(uint8_t), free_store, &free_q);
    full_slots= xQueueCreateStatic(k_max_tickets, sizeof(uint8_t), full_store, &full_q);
    state= xEventGroupCreateStatic(&state_store);
    xEventGroupSetBits(state, k_idle);

    for (uint8_t i = 0; i < k_max_tickets; ++i) xQueueSend(free_slots, &i, 0);

    handle= xTaskCreateStatic(run, "Machine", k_stack_words, this, tskIDLE_PRIORITY, stack, &task);
    THECONVEYOR.wake_on_block(handle);
    vTaskPrioritySet(handle, k_priority);
}

OnMachine MachineTask::proof() const
{
    if(!on_task()) __debugbreak();
    return OnMachine{};
}

void MachineTask::run(void *self)
{
    ((MachineTask *)self)->loop();
}

bool MachineTask::post(Job job, const Gcode &gcode)
{
    uint8_t slot;
    if(!take_slot(slot)) return false;

    ring[slot].kind= Ticket::LINE;
    ring[slot].job= job;
    ring[slot].gcode= gcode;

    publish(slot);
    return true;
}

// blocks until a slot is free. the wait is bounded so the watchdog keeps being fed
bool MachineTask::take_slot(uint8_t &slot)
{
    if(xTaskGetCurrentTaskHandle() == handle) return false;

    while(!halted) {
        if(xQueueReceive(free_slots, &slot, pdMS_TO_TICKS(k_room_wait_ms)) == pdTRUE) {
            watchdog.alive();
            return true;
        }
        watchdog.alive();
    }
    return false;
}

void MachineTask::publish(uint8_t slot)
{
    // together, or the machine task can look between the two and call the machine idle.
    // the scheduler, not the interrupts: both of these are FreeRTOS calls
    vTaskSuspendAll();
    xQueueSend(full_slots, &slot, 0);   // a slot this task owns always fits
    xEventGroupClearBits(state, k_idle);
    xTaskResumeAll();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
}

uint32_t MachineTask::motion_mark() const
{
    return THECONVEYOR.queue_mark();
}

bool MachineTask::motion_passed(uint32_t mark) const
{
    return THECONVEYOR.passed(mark);
}

unsigned int MachineTask::running_line() const
{
    return THECONVEYOR.running_line();
}

bool MachineTask::homed() const
{
    return THEROBOT.is_homed_all_axes();
}

void MachineTask::enforce_keepout()
{
    THEROBOT.set_keepout(true);
}

bool MachineTask::wait_idle(EventBits_t ends)
{
    while(true) {
        EventBits_t bits= xEventGroupGetBits(state);
        if(bits & k_idle) return true;
        if(bits & ends) return false;

        watchdog.alive();
        xEventGroupWaitBits(state, k_idle | ends, pdFALSE, pdFALSE,
                            pdMS_TO_TICKS(k_room_wait_ms));
    }
}

bool MachineTask::post_jog(const float delta[], uint8_t naxis, float scale)
{
    if(xTaskGetCurrentTaskHandle() == handle) return false;
    if(naxis > k_max_actuators || halted) return false;

    // a queued jog would keep moving after the button is let go, so a full ring drops it
    uint8_t slot;
    if(xQueueReceive(free_slots, &slot, 0) != pdTRUE) return false;

    ring[slot].kind= Ticket::JOG;
    for (uint8_t i = 0; i < naxis; ++i) ring[slot].move.delta[i]= delta[i];
    ring[slot].move.naxis= naxis;
    ring[slot].move.scale= scale;

    publish(slot);
    return true;
}

bool MachineTask::post_move(const float delta[], float rate_mm_s)
{
    uint8_t slot;
    if(!take_slot(slot)) return false;

    ring[slot].kind= Ticket::MOVE;
    for (uint8_t i = 0; i < k_max_actuators; ++i) ring[slot].move.delta[i]= i < 3 ? delta[i] : 0;
    ring[slot].move.naxis= 3;
    ring[slot].move.scale= rate_mm_s;

    publish(slot);
    return post_drain();
}

// the boot script may read positions, which only mean anything after homing
static void startup()
{
    THEROBOT.home_on_startup();
    scripts.boot();
}

void MachineTask::post_startup()
{
    if(!post([](Gcode &, OnMachine) { startup(); }, Gcode{}))
        printk("ERROR: no machine task, the machine will not move\n");
}

// true when the machine came to a stop, whatever brought it there
bool MachineTask::post_stop()
{
    vTaskSuspendAll();
    xEventGroupClearBits(state, k_idle);
    stopping= true;
    xTaskResumeAll();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
    return wait_idle(k_halted);
}

bool MachineTask::post_drain()
{
    vTaskSuspendAll();
    xEventGroupClearBits(state, k_idle);
    draining= true;
    xTaskResumeAll();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
    return wait_idle();
}

void MachineTask::drop_all()
{
    uint8_t slot;
    while(xQueueReceive(full_slots, &slot, 0) == pdTRUE) xQueueSend(free_slots, &slot, 0);
}

void MachineTask::serve_tickets()
{
    uint8_t slot;
    while(xQueueReceive(full_slots, &slot, 0) == pdTRUE) {
        // a job that returns into a pending stop must not be followed by the next one: the loop
        // that runs the stop comes after this
        if(interrupted()) {
            xQueueSend(free_slots, &slot, 0);
            drop_all();
            return;
        }

        xEventGroupClearBits(state, k_idle);

        Ticket t= ring[slot];
        xQueueSend(free_slots, &slot, 0);   // the copy is ours, the slot can be refilled

        if(t.kind == Ticket::JOG) {
            THEROBOT.jog_move(t.move.delta, t.move.naxis, t.move.scale);
        } else if(t.kind == Ticket::MOVE) {
            THEROBOT.delta_move_sync(t.move.delta, t.move.scale, t.move.naxis);
        } else {
            t.job(t.gcode, OnMachine{});
        }

    }
}

// the queue pre-load runs off a clock, so the wait has to come back even when nothing wakes it
// what the step ticker is doing, whenever it changes: the numbers a hold is made of. Read here,
// printed from the main loop: a print from the machine task stalls the planner on the console
void MachineTask::trace()
{
    if(!tracing) return;
    static const char *names[]= {"IDLE", "MOVING", "BRAKING", "HELD"};
    StepTicker &t= THEKERNEL->step_ticker;
    int used= 0;
    THECONVEYOR.each_slot([&](Conveyor::Planned p) { if(!p.free) used++; });
    uint8_t m= t.motion();
    bool hold= THEKERNEL->get_feed_hold();
    if(m == traced_motion && used == traced_used && hold == traced_hold) return;
    traced_motion= m; traced_used= used; traced_hold= hold;
    uint32_t now= us_ticker_read();
    uint32_t ms= (now - traced_at) / 1000;
    traced_at= now;

    const Block *b= t.get_current_block();
    if(m == StepTicker::IDLE || b == nullptr) {
        printk("[motion] +%lums %s feed_hold=%d paused=%d slots=%d\n", (unsigned long)ms, names[m], hold, t.paused(), used);
        return;
    }
    uint32_t steps= b->steps_event_count();
    float cap= t.path_rate(); // steps/s, longest axis
    float plateau= steps ? b->ramp.plateau_rate * b->millimeters / steps : 0.f;
    printk("[motion] +%lums %s feed_hold=%d paused=%d resumable=%d slots=%d cap=%.0f | %.3fmm entry=%.2f plateau=%.2f exit=%.2f\n",
           (unsigned long)ms, names[m], hold, t.paused(), t.resumable(), used, cap,
           b->millimeters, b->entry_speed, plateau, b->exit_speed);
}

void MachineTask::tick()
{
    if(!on_task()) __debugbreak();

    // here, not in the loop: a wait for the queue runs the tick itself and cannot end before this
    StepTicker &ticker= THEKERNEL->step_ticker;
    if(ticker.motion() == StepTicker::HELD) {
        if(!ticker.resumable()) {
            THECONVEYOR.flush_queue();
            ticker.release();
        } else if(!THEKERNEL->get_feed_hold()) {
            THEKERNEL->planner.resume_held();
            ticker.release();
        }
    }

    THECONVEYOR.service();
    ulTaskNotifyTakeIndexed(k_notify_index, pdTRUE, pdMS_TO_TICKS(k_poll_ms));
}

// from interrupts too, so it only sets flags and cuts power: no queue, no waiting
void MachineTask::halt(uint8_t why, const char *what)
{
    if(!halted) {
        reason= why;
        strncpy(msg, what != nullptr ? what : "halted", sizeof(msg) - 1);
        msg[sizeof(msg) - 1]= '\0';
    }
    halted= true;
    Killable::kill_all();
    pending= true;
}

void MachineTask::dispatch_halt()
{
    taskENTER_CRITICAL();
    bool report= pending;
    pending= false;
    taskEXIT_CRITICAL();

    if(!report) return;

    printk("ALARM: %s\n", msg);
    Killable::cleanup_all();
}

// not a ticket: a stop must work from any task, and with the ring full
void MachineTask::abort(uint8_t why, const char *what)
{
    halt(why, what);
}

void MachineTask::hold(bool on)
{
    if(!THEKERNEL->is_feed_hold_enabled()) return;
    THEKERNEL->set_feed_hold(on);
    THEKERNEL->step_ticker.hold(on);
    if(tracing) {
        uint32_t now= us_ticker_read();
        printk("[hold] +%lums %s\n", (unsigned long)((now - traced_at) / 1000), on ? "on" : "off");
        traced_at= now;
    }
}

bool MachineTask::unlock(StreamOutput *stream)
{
    if(!halted) return false;
    clear_halt();

    // the caller goes on to home or move, so the machine is clear by the time this returns
    wait_idle();

    stream->printf("[Caution: Unlocked]\n");
    stream->printf("WARNING: After HALT you should HOME as position may currently be unknown\n");
    return true;
}

void MachineTask::clear_halt()
{
    THEKERNEL->set_feed_hold(false);
    THEKERNEL->step_ticker.hold(false);   // a hold from before the halt would keep the ticker from fetching
    Killable::restore_all();

    vTaskSuspendAll();
    xEventGroupClearBits(state, k_idle);
    clearing= true;
    xTaskResumeAll();

    xTaskNotifyGiveIndexed(handle, k_notify_index);
}

void MachineTask::finish_clear()
{
    clearing= false;

    taskENTER_CRITICAL();
    draining= stopping= false;
    taskEXIT_CRITICAL();

    drop_all();
    THECONVEYOR.flush_queue();

    while(THECONVEYOR.flushing()) tick();

    THEROBOT.set_keepout(true);

    // last: a line accepted before this would plan from the old position
    halted= false;
    xEventGroupClearBits(state, k_halted);
}

void MachineTask::loop()
{
    while(true) {
        // halt() runs in interrupts and cannot touch an event group, so the bit follows here
        if(halted) xEventGroupSetBits(state, k_halted);

        dispatch_halt();
        if(clearing) finish_clear();

        // a drain has to run here to set running false, which sends the lookahead to the ticker
        taskENTER_CRITICAL();
        bool drain= draining, stop= stopping;
        draining= stopping= false;
        taskEXIT_CRITICAL();

        // jobs posted ahead of the stop would run after it: they go the way they do on a halt
        if(stop) {
            THECONVEYOR.stop_soon();
            drop_all();
        }
        else if(drain) THECONVEYOR.wait_for_idle();

        serve_tickets();

        vTaskSuspendAll();
        bool quiet= !draining && !stopping && !clearing
                 && uxQueueMessagesWaiting(full_slots) == 0 && THECONVEYOR.is_idle();
        if(quiet) xEventGroupSetBits(state, k_idle);
        if(THEKERNEL->get_feed_hold()) xEventGroupSetBits(state, k_held);
        else xEventGroupClearBits(state, k_held);
        xTaskResumeAll();

        tick();
    }
}
