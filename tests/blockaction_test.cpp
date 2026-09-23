// actions run in order, after the block they were written against, and are dropped on a flush
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>

#include "BlockActions.h"
#include "Gcode.h"

void printk(const char *, ...) {}

static int failures = 0;
static void check(bool ok, const char *what)
{
    if (ok) return;
    printf("FAIL: %s\n", what);
    failures++;
}

static std::vector<std::string> fired;
static void record(void *, Gcode *g)
{
    char buf[32];
    if (g->has_letter('S')) snprintf(buf, sizeof(buf), "M%u S%g", g->m, g->get_value('S'));
    else snprintf(buf, sizeof(buf), "M%u", g->m);
    fired.push_back(buf);
}

static McodeRegistry::Mcode code{0, McodeRegistry::ANY_SUBCODE, McodeRegistry::ACTION, nullptr, record, nullptr};

static Gcode line(const char *text)
{
    return Gcode(std::string(text), nullptr, 0);
}

int main()
{
    // an action waits for the block it was written after, and no longer
    {
        BlockActions a;
        fired.clear();
        a.hold(&code, line("M3 S10000"), 1);
        a.hold(&code, line("M7"), 2);

        a.run_upto(0);
        check(fired.empty(), "nothing runs before its block finishes");

        a.run_upto(1);
        check(fired.size() == 1 && fired[0] == "M3 S10000", "the first action runs with its value");

        a.run_upto(2);
        check(fired.size() == 2 && fired[1] == "M7", "the second runs when its block finishes");
        check(a.empty(), "the list drains");
    }

    // several actions on one block run in the order they were written
    {
        BlockActions a;
        fired.clear();
        a.hold(&code, line("M3 S200"), 5);
        a.hold(&code, line("M7"), 5);
        a.hold(&code, line("M801"), 5);
        a.run_upto(5);
        check(fired.size() == 3, "all three run");
        check(fired[0] == "M3 S200" && fired[1] == "M7" && fired[2] == "M801", "they run in order");
    }

    // a retire that passes several blocks at once still runs everything due
    {
        BlockActions a;
        fired.clear();
        a.hold(&code, line("M3 S1"), 1);
        a.hold(&code, line("M5"), 4);
        a.run_upto(9);
        check(fired.size() == 2, "a late drain runs everything due");
    }

    // the list is bounded, and a refusal tells the caller to run it itself
    {
        BlockActions a;
        for (int i = 0; i < BlockActions::k_max_pending; i++) {
            check(a.hold(&code, line("M7"), 1), "a free slot accepts");
        }
        check(!a.hold(&code, line("M7"), 1), "a full list refuses");
    }

    // a flush throws the actions away with the moves they were written against
    {
        BlockActions a;
        fired.clear();
        a.hold(&code, line("M3 S5000"), 1);
        a.clear();
        a.run_upto(99);
        check(fired.empty(), "a cleared action never runs");
        check(a.empty(), "the list is empty after a clear");
    }

    // a line with more words than fit is refused whole, so the caller runs it with all of them
    {
        BlockActions a;
        fired.clear();
        check(!a.hold(&code, line("M3 S1 X2 Y3 Z4 A5"), 1), "too many words is refused");
        check(a.empty(), "the refused entry is not kept");

        check(a.hold(&code, line("M7"), 1), "the next hold still works");
        a.run_upto(1);
        check(fired.size() == 1 && fired[0] == "M7", "the refusal left no scratch behind");
    }

    // a handler that runs another drain must not see its own action again
    {
        static BlockActions *self;
        static int reentered;
        struct R {
            static void run(void *, Gcode *) {
                fired.push_back("outer");
                reentered++;
                self->run_upto(99);   // what a handler's own wait_for_idle would reach
            }
        };
        static McodeRegistry::Mcode re{0, McodeRegistry::ANY_SUBCODE, McodeRegistry::ACTION, nullptr, R::run, nullptr};

        BlockActions a;
        self = &a;
        reentered = 0;
        fired.clear();
        a.hold(&re, line("M3"), 1);
        a.hold(&re, line("M7"), 1);
        a.run_upto(1);
        check(reentered == 2, "each action runs exactly once");
        check(fired.size() == 2, "a reentrant drain does not double-run");
    }

    printf(failures == 0 ? "all passed\n" : "%d failed\n", failures);
    return failures != 0;
}
