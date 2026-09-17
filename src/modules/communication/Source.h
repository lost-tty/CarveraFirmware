#pragma once

#include "libs/Module.h"

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
    virtual bool internal() const { return false; } // lines leave the modal motion alone and trigger no scripts
};

// Feeds one line of the top source per main loop. Console G-code is refused while a source is active.
// "list [n]" shows where every source stands.
class SourceStack : public Module {
public:
    void on_module_loaded() override;
    void on_main_loop(void *) override;
    void on_halt(void *) override;
    void on_console_line_received(void *) override;
    bool push(Source *s);   // false when already stacked
    void clear();           // aborts every source, top first
    void suspend();         // freezes what is running; a source pushed afterwards still feeds
    void resume();
    Source *top() const { return stack.empty() ? nullptr : stack.back(); }
    bool empty() const { return stack.empty(); }
    bool active() const { return !stack.empty() && !frozen_all(); } // something will feed; MDI would interleave

private:
    bool frozen_all() const;
    std::vector<Source *> stack;
    unsigned frozen= 0;     // depth of the stack when suspended
};

extern SourceStack sources;
