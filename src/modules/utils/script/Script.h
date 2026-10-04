#pragma once

#include "GcodeLine.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// O-word scripts (LinuxCNC subset): o<name> sub/endsub/call/return, oN if/elseif/else/endif, while/endwhile,
// repeat/endrepeat, break, continue. #1..#30 are call arguments, #<name> sub-local, #<_name> global.
// "o<name> abort [reason]" is ours: it ends the whole script and halts the machine with that reason.
namespace script {

// The text of a script, read in place from its files, addressed as one range of offsets. Nothing is
// copied; one file is open at a time, released when a script has finished.
class Source {
public:
    static const unsigned MAX_LINE = 132; // read buffer; longer lines take several reads
    struct Segment {
        uint32_t base, size;
        char *path;
    };
    Source() {}
    ~Source() { clear(); }
    void clear();
    void add(const std::string &path, size_t length);
    void release();                                                       // closes the open file
    unsigned size() const { return segments.empty() ? 0 : segments.back().base + segments.back().size; }
    bool line_at(unsigned offset, std::string &out, unsigned &next);      // the line starting at offset, next: the one after
    unsigned next(unsigned offset);                                       // offset of the line after the one at offset
    bool read(unsigned offset, char *buf, size_t length);
    const char *chunk(unsigned offset, unsigned &length, char *buf, size_t size);
    unsigned line_of(unsigned offset);                                    // 1-based line within its segment
    int segment_of(unsigned offset) const;
    std::string basename(unsigned segment) const;
    std::vector<Segment> segments;

private:
    bool open(int segment);
    Source(const Source &);            // segments own their paths: no copying
    Source &operator=(const Source &);
    FILE *fd = nullptr;
    int opened = -1;
    unsigned long read_at = 0;
};

enum Kind : uint8_t { SUB, ENDSUB, CALL, RETURN, ABORT, IF, ELSEIF, ELSE, ENDIF, WHILE, ENDWHILE, BREAK, CONTINUE, REPEAT, ENDREPEAT };

struct Sub {
    uint32_t offset;
    uint32_t line;   // within its segment
    uint16_t name;   // into Program::names
};

inline bool is_control(const char *p)
{
    return (p[0] == 'o' || p[0] == 'O') && (p[1] == '<' || (p[1] >= '0' && p[1] <= '9'));
}

// Validates the scripts at load and keeps only their subs; the runner finds the blocks as it reads.
class Program {
public:
    bool load(Source &source, std::string &err); // err: "line N: ...", N within the segment at error_offset
    int find_sub(const char *name) const;        // index into subs, or -1
    const char *name(int i) const { return names.c_str() + subs[i].name; }
    static const unsigned MAX_LABEL = 64;

    Source *source = nullptr;
    unsigned error_offset = 0;
    std::vector<Sub> subs;

private:
    typedef std::vector<std::pair<std::string, uint32_t> > Calls;
    bool check(unsigned segment, Calls &calls, std::string &err);
    std::string names;   // each ended by a '\0'
};

// Executes a script one G-code line at a time. Lines come back with parameters substituted.
// Globals (#<_name>) persist from one program to the next.
class Runner {
public:
    Runner(const Program &program, gcode::ParamStore &machine);
    bool start(const char *sub, const float *args, unsigned nargs, std::string &err); // sub null: the main body
    bool start_call(const char *line, std::string &sub, std::string &err);
    bool set_local(const char *name, float v); // a #<name> for the sub just started, e.g. a G-code block's words
    enum Result { LINE, MESSAGE, DONE, ERROR }; // MESSAGE: a (MSG,..) (DEBUG,..) or (PRINT,..) comment, text in out
    Result step(std::string &out, std::string &err);
    unsigned last_offset() const { return current; }                     // of the last LINE or MESSAGE, for trace and list
    unsigned error_offset() const { return failed.at; }                  // where ERROR was raised
    float aborted() const { return abort_reason; } // non-zero after an abort ended the script
    bool running() const { return !frames.empty(); }
    void stop() { while (!frames.empty()) pop(); }
    bool global(const char *name, float &v) const { return store.get_named(name, v); } // #<_name>, e.g. _value

    static const unsigned MAX_DEPTH = 8;
    static const unsigned MAX_ARGS = 30;
    static const unsigned MAX_NAMED = 64;
    static const unsigned MAX_SILENT_STEPS = 10000; // control steps without a line: the script loops forever

private:
    struct Mark {
        uint32_t at, line, next; // a line, its number and the offset of the one after
    };
    struct Loop {
        uint32_t at, line;
        uint32_t left;
        std::string label;
    };
    struct Frame {
        uint32_t at;      // offset of the next line to look at
        uint32_t line;
        uint16_t base;    // its arguments start here in args
        bool testing;     // arrived at an elseif/else because the previous condition was false
        uint32_t has_arg;
        std::vector<Loop> loops; // innermost last
    };
    struct Named {
        uint8_t depth; // frame index, GLOBAL for #<_name>
        std::string name;
        float value;
    };
    static const uint8_t GLOBAL = 0xFF;

    // #1..#30 and #<name> from the frame, #<_name> global, everything else the machine
    class Store : public gcode::ParamStore {
    public:
        explicit Store(Runner &r) : r(r) {}
        bool get(int n, float &v) const override;
        bool set(int n, float v) override;
        bool get_named(const char *name, float &v) const override;
        bool has_named(const char *name) const override;
        bool set_named(const char *name, float v, std::string &err) override;
        Runner &r;
    };

    Result fail(std::string &err, const std::string &msg);
    void reset() { stop(); silent_steps = 0; abort_reason = 0; }
    bool enter(const char *sub, const float *args, unsigned nargs, std::string &err);
    bool call(const std::string &text, std::string &sub, std::string &err);
    bool control(const Mark &here, const std::string &text, std::string &err);
    bool find(const Mark &from, const char *label, size_t length, unsigned kinds, Mark &out,
              std::string &err);
    void jump_to(uint32_t at, uint32_t line)
    {
        frames.back().at = at;
        frames.back().line = line;
    }
    void jump_after(const Mark &m) { jump_to(m.next, m.line + 1); }
    static void advance(Frame &f, unsigned next)
    {
        f.at = next;
        f.line++;
    }
    void took(Frame &f, unsigned at, unsigned next)
    {
        current = at;
        advance(f, next);
    }
    bool arguments(const char *p, float *out, unsigned &n, std::string &err);
    bool substitute(const std::string &text, std::string &out, std::string &err);
    bool message(const std::string &text, std::string &out, std::string &err);
    bool push(int sub, const float *args, unsigned nargs, std::string &err);
    void pop();
    Named *find_named(const char *name, uint8_t depth);

    const Program &program;
    gcode::ParamStore &machine;
    Store store;
    std::vector<Frame> frames;
    std::vector<float> args;  // the top frame's arguments are the tail
    std::vector<Named> named;
    unsigned silent_steps = 0;
    unsigned current = 0;     // offset of the last LINE or MESSAGE
    Mark failed = {0, 0, 0};
    float abort_reason = 0;
};

}
