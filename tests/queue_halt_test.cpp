// Nothing moves once halted, whatever the queue holds and whoever is still producing.
// The conveyor's three indices and their guards are transcribed from Conveyor.cpp; the ring is
// the real BlockQueue shape: one writer per index, no locks.
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

static const unsigned LENGTH = 32;

static volatile bool halted;
static bool flush_flag;
static bool allow_fetch;

struct Ring {
    unsigned head_i = 0;        // written by the main task in queue_head_block
    unsigned isr_tail_i = 0;    // written by the step ISR in get_next_block
    unsigned tail_i = 0;        // written by the main task in on_idle
    bool live[LENGTH] = {};     // a block the ISR was handed

    static unsigned next(unsigned i) { return (i + 1 >= LENGTH) ? 0 : i + 1; }
    bool is_empty() const { return head_i == tail_i; }
    bool is_full() const { return next(head_i) == tail_i; }
} q;

static unsigned steps_issued;   // every step the ticker would have emitted

// Conveyor::queue_head_block, the producer
static bool queue_head_block()
{
    if(halted) return false;    // does not stick more on the queue once halted
    if(q.is_full()) return false;
    q.live[q.head_i] = true;
    q.head_i = Ring::next(q.head_i);
    return true;
}

// Conveyor::get_next_block, called by the step ISR
static bool get_next_block(unsigned *block)
{
    if(flush_flag) q.isr_tail_i = q.head_i;
    if(halted || q.isr_tail_i == q.head_i) return false;
    if(!allow_fetch) return false;
    *block = q.isr_tail_i;
    q.isr_tail_i = Ring::next(q.isr_tail_i);
    return true;
}

// StepTicker::step_tick, reduced to the block handling and the halt guard
static bool ticker_running;
static unsigned ticker_block;
static void step_tick()
{
    if(!ticker_running) {
        if(get_next_block(&ticker_block)) ticker_running = true;
        else return;
    }
    if(halted) {                // the guard that stops motion
        ticker_running = false;
        return;
    }
    steps_issued++;             // only reached when not halted
}

// Conveyor::check_queue, which is what lets the ticker fetch again after a flush
static void check_queue(bool force)
{
    if(q.is_empty()) { allow_fetch = false; return; }
    if(force || q.is_full()) { if(!flush_flag) allow_fetch = true; }
}

// Conveyor::on_idle, reclaiming one block per pass
static void on_idle()
{
    if(q.tail_i != q.isr_tail_i) {
        q.live[q.tail_i] = false;
        q.tail_i = Ring::next(q.tail_i);
    }
}

// Conveyor::flush_queue
static void flush_queue()
{
    allow_fetch = false;
    flush_flag = true;
    for(unsigned guard = 0; !q.is_empty() && guard < LENGTH * 4; guard++) {
        check_queue(true);
        step_tick();            // the ISR keeps firing and advances isr_tail_i
        on_idle();
    }
    flush_flag = false;
}

static void reset_all()
{
    q = Ring{};
    halted = false; flush_flag = false; allow_fetch = true;
    steps_issued = 0; ticker_running = false; ticker_block = 0;
}

int main()
{
    // ---- steps are issued while running ------------------------------------------------------
    {
        reset_all();
        for(int i = 0; i < 5; i++) CHECK(queue_head_block());
        for(int i = 0; i < 20; i++) step_tick();
        CHECK(steps_issued > 0);
    }

    // ---- halting midway through the queue stops the steps immediately ------------------------
    {
        reset_all();
        for(int i = 0; i < 10; i++) queue_head_block();
        for(int i = 0; i < 5; i++) step_tick();
        unsigned before = steps_issued;
        CHECK(before > 0);

        halted = true;
        for(int i = 0; i < 100; i++) step_tick();
        CHECK(steps_issued == before);      // not one more step, with 10 blocks still queued
        CHECK(!q.is_empty());
    }

    // ---- the planner producing concurrently cannot restart motion ----------------------------
    {
        reset_all();
        for(int i = 0; i < 4; i++) queue_head_block();
        for(int i = 0; i < 3; i++) step_tick();
        unsigned before = steps_issued;

        halted = true;
        // the planner is mid-flight and keeps trying to queue while the ticker keeps ticking
        for(int i = 0; i < 50; i++) {
            queue_head_block();             // refused: is_halted checked first
            step_tick();
        }
        CHECK(steps_issued == before);
        CHECK(q.head_i == 4);               // nothing new was accepted
    }

    // ---- a block already handed to the ticker does not keep stepping --------------------------
    {
        reset_all();
        queue_head_block();
        step_tick();                        // ticker now owns a block and is running
        CHECK(ticker_running);
        unsigned before = steps_issued;

        halted = true;
        step_tick();
        CHECK(!ticker_running);             // dropped on the first tick after the halt
        for(int i = 0; i < 50; i++) step_tick();
        CHECK(steps_issued == before);
    }

    // ---- the flush empties the queue even though the ticker refuses to run -------------------
    {
        reset_all();
        for(int i = 0; i < 12; i++) queue_head_block();
        halted = true;

        flush_queue();
        CHECK(q.is_empty());
        CHECK(q.head_i == q.tail_i && q.tail_i == q.isr_tail_i);
        for(unsigned i = 0; i < LENGTH; i++) CHECK(!q.live[i]);   // every block reclaimed
    }

    // ---- flushing a full queue still terminates ----------------------------------------------
    {
        reset_all();
        while(queue_head_block()) {}        // fill it
        CHECK(q.is_full());
        halted = true;

        flush_queue();
        CHECK(q.is_empty());
    }

    // ---- after a flush the ticker stays quiet, and resumes only once unhalted ----------------
    {
        reset_all();
        for(int i = 0; i < 6; i++) queue_head_block();
        halted = true;
        flush_queue();

        unsigned before = steps_issued;
        for(int i = 0; i < 20; i++) step_tick();
        CHECK(steps_issued == before);      // empty queue, still halted

        halted = false;                     // unlocked, but nothing was left to run
        for(int i = 0; i < 20; i++) step_tick();
        CHECK(steps_issued == before);

        CHECK(queue_head_block());          // a fresh move is accepted
        check_queue(true);                  // the conveyor lets the ticker fetch again
        for(int i = 0; i < 5; i++) step_tick();
        CHECK(steps_issued > before);
    }

    printf("%s\n", fails == 0 ? "all passed" : "FAILURES");
    return fails;
}
