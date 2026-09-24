#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "Gcode.h"
#include "ActuatorCoordinates.h"

#include <stdint.h>

enum HALT_REASON {
  // No need to reset when triggered
  MANUAL              = 1,
  HOME_FAIL           = 2,
  PROBE_FAIL          = 3,
  CALIBRATE_FAIL      = 4,
  ATC_HOME_FAIL       = 5,
  ATC_TOOL_INVALID    = 6,
  ATC_NO_TOOL         = 7,
  ATC_HAS_TOOL        = 8,
  SPINDLE_OVERHEATED  = 9,
  SOFT_LIMIT          = 10,
  COVER_OPEN          = 11,
  PROBE_INVALID       = 12,
  E_STOP              = 13,
  NON_HOME            = 15,
  SCRIPT              = 16,
  // Need to reset when triggered
  HARD_LIMIT          = 21,
  MOTOR_ERROR_X       = 22,
  MOTOR_ERROR_Y       = 23,
  MOTOR_ERROR_Z       = 24,
  SPINDLE_STALL       = 25,
  SD_ERROR            = 26,
  // Need to switch off/on the power
  SPINDLE_ALARM       = 41
};

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

    bool post(Job job, const Gcode &gcode);

    // a queued jog would keep moving after the button is let go, so a full ring drops it
    bool post_jog(const float delta[], uint8_t naxis, float scale);

    bool post_move(const float delta[], float rate_mm_s);

    void post_startup();

    // only this task may wait on a block
    bool post_stop();

    bool post_drain();
    bool idle() const { return (xEventGroupGetBits(state) & k_idle) != 0; }
    bool full() const { return uxQueueMessagesWaiting(free_slots) == 0; }

    uint32_t motion_mark() const;
    bool motion_passed(uint32_t mark) const;

    unsigned int running_line() const;

    bool homed() const;
    bool prepare_for_job();
    void enforce_keepout();

    void push_modal_state();
    void pop_modal_state();

    void drop_all();

    void halt(uint8_t reason, const char *msg = nullptr);
    bool is_halted() const { return halted; }
    uint8_t halt_reason() const { return reason; }

    void dispatch_halt();

    void abort(uint8_t reason, const char *msg);

    void hold(bool on);

    bool unlock(StreamOutput *stream);
    void clear_halt();

private:
    // every wait on this task goes through here, so there is one place that sleeps
    void tick();
    friend class Conveyor;   // waits for a block by running a pass of this task's loop


    bool take_slot(uint8_t &slot);
    void publish(uint8_t slot);

    bool wait_idle(EventBits_t ends = k_halted | k_held);

    static void run(void *);
    void loop();
    void serve_tickets();
    void finish_clear();

    static const uint16_t k_stack_words = 768;
    static const UBaseType_t k_priority = 2;   // above the main loop, below the tickers
    static const UBaseType_t k_notify_index = 1;
    static const uint32_t k_poll_ms = 10;   // the conveyor waits on the same notification
    static const uint32_t k_room_wait_ms = 10;   // short enough to keep feeding the watchdog
    static const EventBits_t k_idle = 1 << 0;
    static const EventBits_t k_halted = 1 << 1;
    static const EventBits_t k_held = 1 << 2;
    static const uint8_t k_max_tickets = 4;

    struct Jog {
        float delta[k_max_actuators];
        float scale;
        uint8_t naxis;
    };

    // the line is copied: the dispatcher's is gone by the time this runs. a jog carries a
    // delta instead, to keep out of the modal state a program is using
    struct Ticket {
        enum Kind : uint8_t { LINE, JOG, MOVE } kind;
        Job job;
        Gcode gcode;
        Jog move;
    };

    Ticket ring[k_max_tickets];
    QueueHandle_t free_slots{nullptr}, full_slots{nullptr};
    StaticQueue_t free_q, full_q;
    uint8_t free_store[k_max_tickets], full_store[k_max_tickets];

    volatile bool clearing{false};
    volatile bool halted{false};
    volatile bool pending{false};
    uint8_t reason{0};
    char msg[32]{};
    volatile bool draining{false};
    volatile bool stopping{false};

    EventGroupHandle_t state{nullptr};
    StaticEventGroup_t state_store;

    StackType_t stack[k_stack_words];
    StaticTask_t task;
    TaskHandle_t handle{nullptr};
};

extern MachineTask machine_task;
