#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "libs/Killable.h"

#include <string>
#include <vector>

struct SerialMessage;
class StreamOutput;

// Something that supplies G-code lines: a job file, a running script. Sources stack, the top one feeds;
// a source pushed while another runs pauses it until it is done.
class Source {
public:
    enum Result { LINE, WAIT, DONE }; // WAIT: nothing to dispatch this loop
    virtual Result next(SerialMessage &msg) = 0;
    virtual void abort() = 0;                        // the job ended under it or the machine halted
    virtual void list(StreamOutput *stream, unsigned around) = 0; // the lines around the current one, like a debugger
};

// Feeds one line of the top source per main loop. Console G-code is refused while a source is active.
// "list [n]" shows where every source stands.
class SourceStack : public Module, public Killable {
public:
    void on_module_loaded() override;
    void on_main_loop(void *) override;
    void kill() override {}
    void cleanup() override;
    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
    SimpleShell::Registered shell_slot;
    bool push(Source *s);   // false when already stacked
    void clear();           // aborts every source, top first
    void suspend();
    void resume();
    Source *top() const { return stack.empty() ? nullptr : stack.back(); }
    bool empty() const { return stack.empty(); }
    bool active() const { return !stack.empty() && !frozen_all(); } // something will feed; MDI would interleave
    bool suspended() const { return frozen != 0; }

    // a line refused while earlier moves are still queued: they finish, then the job stops
    bool stop_after_queued(unsigned int line);

private:
    bool frozen_all() const;
    std::vector<Source *> stack;
    unsigned frozen= 0;     // depth of the stack when suspended
    uint32_t stop_after= 0; // the queue mark the refused line was written before
    unsigned int stop_line= 0;
    bool stopping= false;
};

extern SourceStack sources;
