#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "libs/Killable.h"
#include "GcodeDispatch.h"
#include "Macros.h"

#include <deque>
#include <string>

struct SerialMessage;

class Program : public Module, public Killable {
public:
    void on_module_loaded() override;
    void kill() override {}
    void cleanup() override;
    static void list_shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static void trace_shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static void macro_shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static const SimpleShell::Sub<Program> MACRO_SUBS[];
    void macro_list(std::string args, StreamOutput *stream);
    void macro_params(std::string args, StreamOutput *stream);
    SimpleShell::Registered list_slot, trace_slot, macro_slot;

    Macros &macros() { return files; }
    bool start_job(const std::string &path, std::string &err);
    bool call(const char *sub, const float *args, unsigned nargs, StreamOutput *reply,
              std::string &err);
    bool call_line(const std::string &line, StreamOutput *reply, std::string &err);
    bool run_sub(const char *sub);   // false: there is none or it cannot start
    // false: the code stays with its handler; err: the sub did not start
    bool remap(const char *sub, const gcode::Words &words, std::string &err);

    bool step(SerialMessage &msg);   // true: msg holds a line to dispatch
    void refused(unsigned line);
    bool yields() const { return !in_sub(); }
    bool takes_console() const { return !playing() || paused; }
    bool busy() const { return ending || runner->running(); }
    bool in_sub() const { return runner->running() && !runner->at_main(); }
    bool playing() const { return source.job() >= 0; }
    const char *job_name() const;
    unsigned job_size() const;
    unsigned job_read() const;

    bool suspended() const { return paused; }
    void suspend();
    void resume();
    void ask_pause() { pause_asked= playing(); }  // at the next job line
    bool cancel_pause();
    bool jump(unsigned line, std::string &err);
    void stop();            // the job and any sub on it, or a sub alone

    // a line refused while earlier moves are still queued: they finish, then the job stops
    bool stop_after_queued(unsigned int line);

private:
    bool can_call(std::string &err) const;
    bool called(const std::string &sub, StreamOutput *reply, bool on_job);
    bool advance(SerialMessage &msg);
    bool frozen() const { return paused && !in_sub(); }
    void finish();
    void forget();
    void end_job();
    void halt(int reason);
    void list(StreamOutput *stream, int segment, unsigned current, unsigned around);

    Macros files;
    script::Source source;
    script::Library library;
    script::Runner *runner= nullptr;
    StreamOutput *reply= nullptr;       // caller waiting for ok/error
    std::string name;                   // what runs, for messages
    unsigned line= 0;                   // the last job line read
    uint32_t stop_after= 0;             // the queue mark the refused line was written before
    unsigned int stop_line= 0;
    bool trace= false;
    bool nested= false;                 // a sub runs on the job
    bool ending= false;                 // the job is read to its end, the machine finishes it
    bool paused= false;
    bool pause_asked= false;
    bool stopping= false;
};

extern Program program;
