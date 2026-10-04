#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "libs/Killable.h"
#include "GcodeDispatch.h"
#include "Macros.h"

#include <deque>
#include <string>

struct SerialMessage;

// Runs what the machine executes, one line per main loop: the job as the bottom frame and the subs
// it calls or triggers on top of it, or a sub alone. Console G-code is refused while a line will
// be fed.
class Program : public Module, public Killable {
public:
    void on_module_loaded() override;
    void on_main_loop(void *) override;
    void kill() override {}
    void cleanup() override;
    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
    SimpleShell::Registered shell_slot;

    Macros &macros() { return library; }
    bool start_job(const std::string &path, bool echo, std::string &err);
    bool call(const char *sub, const float *args, unsigned nargs, StreamOutput *reply,
              std::string &err);
    bool call_line(const std::string &line, StreamOutput *reply, std::string &err);
    bool set_local(const char *name, float v) { return runner->set_local(name, v); }
    void restore_modal_on_return(); // of the sub just called: the caller's group 1 comes back
    void set_trace(bool on) { trace= on; }

    bool busy() const { return ending || runner->running(); }
    bool active() const { return busy() && !frozen(); } // something will feed; MDI would interleave
    bool in_sub() const { return runner->running() && !runner->at_main(); }
    bool playing() const { return library.source().job() >= 0; }
    const char *job_name() const;
    unsigned job_size() const;
    unsigned job_read() const;

    bool suspended() const { return paused; }
    void suspend();
    void resume();
    void ask_pause() { pause_asked= playing(); }  // at the next job line
    bool cancel_pause();
    bool insert(const std::string &line);       // fed before the next job line
    bool jump(unsigned line, std::string &err);
    void stop();            // the job and any sub on it, or a sub alone

    // a line refused while earlier moves are still queued: they finish, then the job stops
    bool stop_after_queued(unsigned int line);

private:
    bool can_call(std::string &err) const;
    bool called(const std::string &sub, StreamOutput *reply, bool on_job);
    bool next(SerialMessage &msg);
    bool frozen() const { return paused && !in_sub(); }
    void finish();
    void end_job();
    void halt(int reason);
    void list(StreamOutput *stream, int segment, unsigned current, unsigned around);

    Macros library;
    script::Runner *runner= nullptr;
    StreamOutput *reply= nullptr;       // caller waiting for ok/error
    std::string name;                   // what runs, for messages
    unsigned line= 0;                   // the last job line read
    std::deque<std::string> inserted;
    static const unsigned INSERT_LIMIT = 32; // a remote client must not grow it without bound
    uint32_t stop_after= 0;             // the queue mark the refused line was written before
    unsigned int stop_line= 0;
    bool trace= false;                  // echo every sub line with its origin
    bool echo= false;                   // echo every job line
    bool nested= false;                 // a sub runs on the job
    bool ending= false;                 // the job is read to its end, the machine finishes it
    bool paused= false;
    bool pause_asked= false;
    bool stopping= false;
    bool modal_saved= false;
    GcodeDispatch::ModalState saved_modal;
};

extern Program program;
