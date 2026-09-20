// The halt state machine: who runs what, in which order, and what a second halt or an early
// unlock does. Killable is the real class; Kernel::halt/dispatch_halt/clear_halt are transcribed
// from Kernel.cpp because pulling the kernel in would drag the whole firmware.
#include "Killable.h"

#include <cstdio>
#include <string>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

// ---- the machine state the kernel owns -------------------------------------------------------
static volatile bool halted;
static volatile bool halt_pending;
static uint8_t halt_reason;
static std::vector<std::string> trace;

// Kernel::halt, verbatim in order
static void halt(uint8_t reason)
{
    if(!halted) halt_reason = reason;
    halted = true;
    Killable::kill_all();
    halt_pending = true;
}

// Kernel::dispatch_halt, called from the main loop
static void dispatch_halt()
{
    if(!halt_pending) return;
    halt_pending = false;
    Killable::cleanup_all();
    trace.push_back("on_halt(stop)");
}

// Kernel::clear_halt
static void clear_halt()
{
    dispatch_halt();            // the queue and source stack must be cleared first
    halted = false;             // call_event(ON_HALT, 1) sets this
    trace.push_back("on_halt(clear)");
    Killable::restore_all();
}

static void reset_machine()
{
    halted = false; halt_pending = false; halt_reason = 0;
    trace.clear();
}

// ---- devices ---------------------------------------------------------------------------------
struct Device : public Killable {
    std::string name;
    int kills = 0, cleanups = 0, restores = 0;
    bool output_live = true;

    explicit Device(const char *n) : name(n) {}
    void kill() override    { kills++;    output_live = false; trace.push_back(name + ".kill"); }
    void cleanup() override { cleanups++;               trace.push_back(name + ".cleanup"); }
    void restore() override { restores++; output_live = true;  trace.push_back(name + ".restore"); }
};

// a device whose kill cannot reach the hardware, like the modbus VFD
struct DeferredDevice : public Device {
    explicit DeferredDevice(const char *n) : Device(n) {}
    void kill() override    { kills++; trace.push_back(name + ".kill"); }   // output stays live
    void cleanup() override { cleanups++; output_live = false; trace.push_back(name + ".cleanup"); }
};

int main()
{
    // ---- a halt kills every registered output before anything else ---------------------------
    {
        reset_machine();
        Device spindle("spindle"), laser("laser"), rail("rail");

        halt(13 /* E_STOP */);

        CHECK(halted);
        CHECK(halt_reason == 13);          // the reason is set before the flag
        CHECK(halt_pending);
        CHECK(spindle.kills == 1 && laser.kills == 1 && rail.kills == 1);
        CHECK(!spindle.output_live && !laser.output_live && !rail.output_live);
        // nothing has cleaned up yet: that waits for the main loop
        CHECK(spindle.cleanups == 0 && laser.cleanups == 0);
    }

    // ---- the reason is readable the instant the flag is set -----------------------------------
    // (the old code called set_halt_reason after the broadcast, so handlers saw the previous one)
    {
        reset_machine();
        struct Observer : public Killable {
            uint8_t seen_reason = 0; bool seen_halted = false;
            void kill() override { seen_reason = halt_reason; seen_halted = halted; }
        } obs;

        halt(9 /* SPINDLE_OVERHEATED */);
        CHECK(obs.seen_reason == 9);
        CHECK(obs.seen_halted);
    }

    // ---- cleanup runs once, on the main loop, after the kill ---------------------------------
    {
        reset_machine();
        Device d("d");

        halt(1);
        CHECK(trace == std::vector<std::string>{"d.kill"});

        dispatch_halt();
        CHECK(d.cleanups == 1);
        CHECK(trace == (std::vector<std::string>{"d.kill", "d.cleanup", "on_halt(stop)"}));

        dispatch_halt();                    // a second pass does nothing
        CHECK(d.cleanups == 1);
    }

    // ---- a halt is never conditional: every call kills again ---------------------------------
    {
        reset_machine();
        Device d("d");

        halt(13);
        d.output_live = true;               // something re-enabled the output
        halt(13);                           // pressing e-stop again must kill it
        CHECK(d.kills == 2);
        CHECK(!d.output_live);

        // a halt raised while stopping does not overwrite what caused it
        halt(2 /* HOME_FAIL */);
        CHECK(halt_reason == 13);
    }

    // ---- an unlock runs the pending cleanup first: the queue must not survive it -------------
    {
        reset_machine();
        Device d("d");

        halt(1);                            // e-stop, cleanup queued
        clear_halt();                       // M999 arrives before the main loop turns
        CHECK(d.cleanups == 1);             // flush_queue and the source clear still happened
        CHECK(!halted);
        CHECK(!halt_pending);

        dispatch_halt();                    // and it does not run twice
        CHECK(d.cleanups == 1);
    }

    // ---- restore gives back only what kill took ----------------------------------------------
    {
        reset_machine();
        Device rail("rail");

        halt(13);
        dispatch_halt();
        CHECK(!rail.output_live);

        clear_halt();
        CHECK(rail.restores == 1);
        CHECK(rail.output_live);
        CHECK(!halted);
        // halted is already false when a device restores
        CHECK(trace.back() == "rail.restore");
        CHECK(trace[trace.size() - 2] == "on_halt(clear)");
    }

    // ---- a device that cannot be killed from an interrupt stops in cleanup --------------------
    {
        reset_machine();
        DeferredDevice vfd("vfd");

        halt(13);
        CHECK(vfd.kills == 1);
        CHECK(vfd.output_live);             // the modbus command has not been sent yet

        dispatch_halt();
        CHECK(!vfd.output_live);            // cleanup reached it
    }

    // ---- a destroyed device leaves the list kill_all walks -----------------------------------
    {
        reset_machine();
        Device keep("keep");
        {
            Device temporary("temporary");
            halt(1);
            CHECK(temporary.kills == 1);
            CHECK(keep.kills == 1);
        }
        reset_machine();
        halt(1);                            // must not touch the destroyed device
        CHECK(keep.kills == 2);
        CHECK(trace == std::vector<std::string>{"keep.kill"});
    }

    printf("%s\n", fails == 0 ? "all passed" : "FAILURES");
    return fails;
}
