#pragma once

#include "SimpleShell.h"

#include "libs/Module.h"
#include "libs/Killable.h"
#include "GcodeDispatch.h"
#include "Macros.h"
#include "libs/SerialMessage.h"

#include <deque>
#include <string>

class Program : public Module, public Killable {
public:
    void on_module_loaded() override;
    void kill() override {}
    void cleanup() override;
    static void trace_shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static void macro_shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static const SimpleShell::Sub<Program> MACRO_SUBS[];
    void macro_list(std::string args, StreamOutput *stream);
    void macro_params(std::string args, StreamOutput *stream);
    SimpleShell::Registered trace_slot, macro_slot;

    Macros &macros() { return files; }
    bool load_job(const std::string &path, std::string &err);   // waits for resume
    bool call(const char *sub, const float *args, unsigned nargs, StreamOutput *reply,
              std::string &err);
    bool call_line(const std::string &line, StreamOutput *reply, std::string &err);
    bool run_sub(const char *sub);   // false: there is none or it cannot start
    // false: the code stays with its handler; err: the sub did not start
    bool remap(const char *sub, const gcode::Words &words, std::string &err);

    bool step(SerialMessage &msg);   // true: msg holds a line to dispatch
    unsigned played_line();          // the job line the machine has reached
    std::string place(uint32_t mark) const;   // "file.ngc:12", empty for none
    struct Head {
        const char *path;
        uint32_t size;
        unsigned line;      // the machine's
        unsigned read;      // the program's
        bool job;
        int frame;          // the runner's, -1: returned
    };
    std::string args(const Head &h) const { return h.frame < 0 ? "" : runner->frame_args(h.frame); }
    unsigned heads(Head *out);   // the job's first, then each sub the machine is in
    enum Outcome { DONE, STOPPED, HALTED };
    void refused(uint32_t mark);
    void park(const SerialMessage &msg);   // the line waits for the machine, then goes again
    bool yields() const { return !in_sub(); }
    bool takes_console() const { return !playing() || pause != NONE; }
    bool busy() const { return ending || runner->running(); }
    bool in_sub() const { return runner->running() && !runner->at_main(); }
    bool playing() const { return source.job() >= 0; }
    const char *job_name() const;
    unsigned job_size() const;
    unsigned job_read() const;

    bool suspended() const { return pause != NONE; }
    bool loaded() const { return waiting; }
    bool stepping() const { return stepper.stop_depth != 0; }
    void suspend();
    void resume();
    enum Step { INTO, OVER, OUT };
    void step(Step how);   // resume, pause before the next line on that level
    void ask_pause() { pause_asked= playing(); }  // at the next job line
    bool cancel_pause();
    bool jump(unsigned line, std::string &err);
    void stop();            // the job and any sub on it, or a sub alone

    // a line refused while earlier moves are still queued: they finish, then the job stops
    bool stop_after_queued(uint32_t mark);

private:
    bool can_call(std::string &err) const;
    bool called(const std::string &sub, StreamOutput *reply, bool on_job);
    bool advance(SerialMessage &msg);
    bool frozen() const { return pause == ALL || (pause == JOB && !in_sub()); }
    void finish();
    void stop_at(uint32_t at);
    void note_calls();
    unsigned chain(uint32_t mark, uint32_t *out) const;   // the mark and its callers up to the job
    unsigned depth_of(uint32_t mark) const;   // as the runner's depth was for it
    void follow_fence();
    void lift_fence();
    void release();   // the hold, and a loaded job's wait
    uint32_t standing() const;   // the line the job stands before, else the machine's
    // from the line paused before, else the machine's; the console's outermost is the job's
    unsigned shown_chain(uint32_t *out);
    static unsigned call_of(uint32_t mark) { return mark >> 24; }
    static unsigned line_of(uint32_t mark) { return mark & 0xFFFFFF; }
    uint32_t mark_of(unsigned level, unsigned line) const
    {
        return (runner->entered(level) & 0xFF) << 24 | line;
    }
    int file_of(uint32_t mark) const;
    int frame_of(uint32_t mark) const;
    std::string file_line(int segment, unsigned line) const;
    void forget();
    void end_job();
    void halt(int reason);

    Macros files;
    script::Source source;
    script::Library library;
    script::Runner *runner= nullptr;
    StreamOutput *reply= nullptr;       // caller waiting for ok/error
    std::string name;                   // what runs, for messages
    unsigned played= 0;
    // the calls by number, a mark's top byte; 0 is the job
    struct Call {
        uint32_t entered;   // the runner's number for it
        int8_t segment;
        uint32_t from;      // mark of the calling line, 0: the console
    };
    static const unsigned CALLS = 32;   // more than the motion queue can hold at once
    Call calls[CALLS];
    uint32_t stop_after= 0;             // the queue mark the refused line was written before
    uint32_t stop_mark= 0;
    bool trace= false;
    static const unsigned EVERY = ~0u;
    struct Stepper {
        unsigned stop_depth= 0;                        // pause before a line at most this deep
        SerialMessage held{nullptr, "", 0, nullptr};   // the line paused before
        bool fencing= false;                           // asked the conveyor for a fence
    };
    Stepper stepper;
    bool holding() const { return stepper.held.mark != 0; }
    bool stops_before(unsigned depth) const { return depth <= stepper.stop_depth; }
    bool waiting= false;
    bool nested= false;                 // a sub runs on the job
    bool ending= false;                 // the job is read to its end, the machine finishes it
    // JOB holds the job, a sub called on top of it still runs; ALL holds everything
    enum Pause : uint8_t { NONE, JOB, ALL };
    Pause pause= NONE;
    bool pause_asked= false;
    bool stopping= false;
};

extern Program program;
