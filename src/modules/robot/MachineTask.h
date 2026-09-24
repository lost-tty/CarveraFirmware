#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include "Gcode.h"
#include "ActuatorCoordinates.h"

#include <stdint.h>

// a function taking this cannot be called from the main task: only MachineTask can construct it
class OnMachine
{
    friend class MachineTask;
    OnMachine() = default;
public:
    OnMachine(const OnMachine &) = default;
};

class MachineTask
{
public:
    void start();
    size_t stack_unused() const
    {
        if(handle == nullptr) return 0;
        return uxTaskGetStackHighWaterMark(handle) * sizeof(StackType_t);
    }

    using Job = void (*)(Gcode &gcode, OnMachine);

    bool on_task() const { return handle != nullptr && xTaskGetCurrentTaskHandle() == handle; }

    // asserts, because asking for it off this task is a programming error
    OnMachine proof() const;

    using Ref = uint32_t;
    static const Ref k_no_ticket = 0;

    Ref post(Job job, const Gcode &gcode);

    void wait_for(Ref ticket);
    bool done(Ref ticket) const { return ticket == k_no_ticket || served >= ticket; }

    // a queued jog would keep moving after the button is let go, so a full ring drops it
    bool post_jog(const float delta[], uint8_t naxis, float scale);

    void post_startup();

    // only this task may wait on a block
    bool post_stop();

    bool post_drain();
    bool idle() const { return count == 0; }
    void drop_all();
    void clear_halt();

private:
    // every wait on this task goes through here, so there is one place that sleeps
    void tick();
    friend class Conveyor;   // waits for a block by running a pass of this task's loop

    void wait_for_room();
    void wake_waiter();

    static void run(void *);
    void loop();
    void serve_tickets();
    void finish_clear();

    static const uint16_t k_stack_words = 768;
    static const UBaseType_t k_priority = 2;   // above the main loop, below the tickers
    static const UBaseType_t k_notify_index = 1;
    static const uint32_t k_poll_ms = 10;   // the conveyor waits on the same notification
    static const UBaseType_t k_room_notify_index = 2;
    // a wakeup raised before the waiter is published is lost, so this also bounds that stall
    static const uint32_t k_room_wait_ms = 10;
    static const uint8_t k_max_tickets = 4;

    struct Jog {
        float delta[k_max_actuators];
        float scale;
        uint8_t naxis;
    };

    // the line is copied: the dispatcher's is gone by the time this runs. a jog carries a
    // delta instead, to keep out of the modal state a program is using
    struct Ticket {
        Ref ticket;
        enum Kind : uint8_t { LINE, JOG } kind;
        Job job;
        Gcode gcode;
        Jog move;
    };
    Ticket ring[k_max_tickets];
    volatile uint8_t head{0}, tail{0}, count{0};

    volatile bool clearing{false};

    Ref posted{k_no_ticket};
    volatile Ref served{k_no_ticket};

    volatile TaskHandle_t waiter{nullptr};


    StackType_t stack[k_stack_words];
    StaticTask_t task;
    TaskHandle_t handle{nullptr};
};

extern MachineTask machine_task;
