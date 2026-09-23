// the M code registry: one owner per code, exact subcode shadows ANY, duplicates refused
#include <cstdio>
#include <cstdint>
#include <cstdarg>
#include <string>

#include "McodeRegistry.h"

// the registry reports a duplicate through printk; the test reads what it said
static std::string reported;
void printk(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    reported += buf;
}

using Mcode = McodeRegistry::Mcode;

static int failures = 0;

static void check(bool ok, const char *what)
{
    if (ok) return;
    printf("FAIL: %s\n", what);
    failures++;
}

static void hit_a(void *, Gcode *) {}
static void hit_b(void *, Gcode *) {}

// the registry is one static list, so each case takes code numbers no other case uses
static void shadowing()
{
    static Mcode any, exact;
    McodeRegistry::add(any, 493, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    McodeRegistry::add(exact, 493, 1, McodeRegistry::IMMEDIATE, nullptr, hit_b);
    check(McodeRegistry::find(493, 0)->handler == hit_a, "M493 goes to the ANY owner");
    check(McodeRegistry::find(493, 2)->handler == hit_a, "M493.2 goes to the ANY owner");
    check(McodeRegistry::find(493, 1)->handler == hit_b, "M493.1 goes to the exact owner");

    // the other registration order must give the same answers
    static Mcode exact2, any2;
    McodeRegistry::add(exact2, 494, 1, McodeRegistry::IMMEDIATE, nullptr, hit_b);
    McodeRegistry::add(any2, 494, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    check(McodeRegistry::find(494, 0)->handler == hit_a, "reversed: M494 goes to the ANY owner");
    check(McodeRegistry::find(494, 1)->handler == hit_b, "reversed: M494.1 goes to the exact owner");
}

static void unclaimed()
{
    static Mcode slot;
    McodeRegistry::add(slot, 105, McodeRegistry::ANY_SUBCODE, McodeRegistry::BESIDE_JOB, nullptr, hit_a);
    check(McodeRegistry::find(8, 0) == nullptr, "M8 has no owner");
    check(McodeRegistry::find(105, 0) != nullptr, "M105 has one");
}

// a switch claims each spelling, so a subcode it did not ask for stays unowned
static void switch_codes()
{
    static Mcode on, off;
    McodeRegistry::add(on, 7, 0, McodeRegistry::ACTION, nullptr, hit_a);
    McodeRegistry::add(off, 9, 0, McodeRegistry::ACTION, nullptr, hit_b);
    check(McodeRegistry::find(7, 0)->handler == hit_a, "M7 turns air on");
    check(McodeRegistry::find(9, 0)->handler == hit_b, "M9 turns air off");
    check(McodeRegistry::find(7, 1) == nullptr, "M7.1 has no owner");
}

// the temperature pool claims one slot per distinct report code, and a code another owner
// already took must not eat a slot: the pool sizes its budget to one per member
static void pool_dedupe()
{
    static Mcode taken;
    McodeRegistry::add(taken, 207, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_b);

    reported.clear();
    static Mcode slots[4];
    size_t used = 0;
    uint16_t asked[] = {205, 205, 206, 207};
    for (uint16_t code : asked) {
        bool claimed = false;
        for (size_t i = 0; i < used; i++) {
            if (slots[i].number == code) claimed = true;
        }
        if (claimed) continue;
        if (McodeRegistry::add(slots[used], code, McodeRegistry::ANY_SUBCODE, McodeRegistry::BESIDE_JOB, nullptr, hit_a))
            used++;
    }
    check(used == 2, "a refused code does not eat a slot");
    check(reported == "ERROR: M207 is already claimed\n", "the taken code is named");
    check(McodeRegistry::find(205, 0)->handler == hit_a, "M205 resolves to the pool");
    check(McodeRegistry::find(206, 0)->handler == hit_a, "M206 resolves to the pool");
    check(McodeRegistry::find(207, 0)->handler == hit_b, "M207 stays with its first owner");
}

static void mid_job()
{
    static Mcode beside, barrier;
    McodeRegistry::add(beside, 220, McodeRegistry::ANY_SUBCODE, McodeRegistry::BESIDE_JOB, nullptr, hit_a);
    McodeRegistry::add(barrier, 490, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_b);
    check((McodeRegistry::find(220, 0)->when & McodeRegistry::MID_JOB) != 0, "M220 is allowed mid job");
    check((McodeRegistry::find(490, 0)->when & McodeRegistry::MID_JOB) == 0, "M490 is not");
    check((McodeRegistry::find(220, 0)->when & ~McodeRegistry::MID_JOB) == McodeRegistry::IMMEDIATE, "M220 runs immediately");
}

// re-registering one slot is ignored, so a second config pass does not corrupt the list
static void same_slot()
{
    reported.clear();
    static Mcode slot;
    McodeRegistry::add(slot, 600, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    McodeRegistry::add(slot, 600, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    check(McodeRegistry::find(600, 0) == &slot, "the slot still owns the code");
    check(reported.empty(), "re-registering a slot is silent");
}

// a second owner is refused and named, and the first keeps the code
static void duplicates()
{
    reported.clear();
    static Mcode first, second;
    McodeRegistry::add(first, 561, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    McodeRegistry::add(second, 561, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_b);
    check(reported == "ERROR: M561 is already claimed\n", "the duplicate is named");
    check(McodeRegistry::find(561, 0)->handler == hit_a, "the first owner keeps the code");
    check(second.next == nullptr, "the refused slot is not threaded");

    reported.clear();
    static Mcode sub_first, sub_second;
    McodeRegistry::add(sub_first, 106, 2, McodeRegistry::ACTION, nullptr, hit_a);
    McodeRegistry::add(sub_second, 106, 2, McodeRegistry::ACTION, nullptr, hit_b);
    check(reported == "ERROR: M106.2 is already claimed\n", "the subcode is named");
    check(McodeRegistry::find(106, 2)->handler == hit_a, "the first subcode owner keeps it");

    reported.clear();
    static Mcode any, exact;
    McodeRegistry::add(any, 700, McodeRegistry::ANY_SUBCODE, McodeRegistry::BARRIER, nullptr, hit_a);
    McodeRegistry::add(exact, 700, 1, McodeRegistry::IMMEDIATE, nullptr, hit_b);
    check(reported.empty(), "an exact subcode beside ANY is allowed");
}

int main()
{
    shadowing();
    unclaimed();
    switch_codes();
    pool_dedupe();
    mid_job();
    same_slot();

    duplicates();

    printf(failures == 0 ? "all passed\n" : "%d failed\n", failures);
    return failures != 0;
}
