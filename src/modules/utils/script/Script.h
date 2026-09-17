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

// The text of a script, read in place: segments in flash or files in a directory, addressed as one range of
// offsets (at most 64 KB). Nothing is copied; one file is open at a time, released when a script has finished.
class Source {
public:
    static const unsigned MAX_LINE = 132; // read buffer; longer lines take several reads
    struct Segment { uint16_t base, size; const char *flash; const char *name; uint8_t name_length; uint8_t owned; }; // flash null: file dir/name
    Source() {}
    explicit Source(const char *text); // one segment in memory, for tests
    Source(const char *text, size_t length);
    ~Source() { clear(); }
    void clear();
    bool add(const char *flash, size_t length, const char *name, uint8_t name_length);   // name must outlive the source
    bool add_file(const char *dir, const std::string &name, size_t length);              // dir must outlive the source, the name is copied
    void release();                                                       // closes the open file
    unsigned size() const { return segments.empty() ? 0 : segments.back().base + segments.back().size; }
    bool line_at(unsigned offset, std::string &out, unsigned &next);      // the line starting at offset, next: the one after
    unsigned next(unsigned offset);                                       // offset of the line after the one at offset
    bool read(unsigned offset, char *buf, size_t length);
    const char *chunk(unsigned offset, unsigned &length, char *buf, size_t size); // bytes at offset: flash in place, file via buf
    unsigned line_of(unsigned offset);                                    // 1-based line within its segment
    int segment_of(unsigned offset) const;
    std::string name(unsigned segment) const { return std::string(segments[segment].name, segments[segment].name_length); }
    std::vector<Segment> segments;
    static unsigned opens; // files opened since the last reset, for tests

private:
    bool open(int segment);
    Source(const Source &);            // segments own names: no copying
    Source &operator=(const Source &);
    const char *dir = nullptr;
    FILE *fd = nullptr;
    int opened = -1;
};

enum Kind : uint8_t { SUB, ENDSUB, CALL, RETURN, ABORT, IF, ELSEIF, ELSE, ENDIF, WHILE, ENDWHILE, BREAK, CONTINUE, REPEAT, ENDREPEAT };

struct Control {
    uint16_t offset; // of the line in the source
    Kind kind;
    uint8_t arg;     // offset of the argument text in the trimmed line
    uint16_t label;  // index into the label list
    uint16_t match;  // sub<->endsub, if/elseif/else->next branch or endif, while<->endwhile, repeat<->endrepeat,
                     // break/continue->loop, call->sub
};

struct Label { uint16_t offset, length; }; // the name as written in the source; numeric labels compare by value

// A validated script: control words with their pairing. Everything else is checked to tokenize.
class Program {
public:
    bool load(Source &source, std::string &err); // err: "line N: ...", N within the segment at error_offset
    int find_sub(const char *name) const;        // index into controls, or -1
    const Control *control_at(unsigned offset) const;
    unsigned after(const Control &c) const { return source->next(c.offset); }
    bool label_is(const Label &l, const char *name, size_t length) const;
    std::string label_text(const Label &l) const;
    static const unsigned MAX_LABEL = 64;

    Source *source = nullptr;
    unsigned error_offset = 0;
    std::vector<Control> controls; // by offset
    std::vector<Label> labels;
};

// Executes a Program one G-code line at a time. Lines come back with parameters substituted.
// Globals (#<_name>) persist across start() so scripts can keep state between calls.
class Runner {
public:
    Runner(const Program &program, gcode::ParamStore &machine);
    bool start(const char *sub, const float *args, unsigned nargs, std::string &err); // sub null: the main body
    bool set_local(const char *name, float v); // a #<name> for the sub just started, e.g. a G-code block's words
    enum Result { LINE, MESSAGE, DONE, ERROR }; // MESSAGE: a (MSG,..) (DEBUG,..) or (PRINT,..) comment, text in out
    Result step(std::string &out, std::string &err);
    unsigned last_offset() const { return current; }                     // of the last LINE or MESSAGE, for trace and list
    unsigned error_offset() const { return failed_at; }                  // where ERROR was raised
    float aborted() const { return abort_reason; } // non-zero after an abort ended the script
    bool running() const { return !frames.empty(); }
    void stop() { while (!frames.empty()) pop(); }
    bool global(const char *name, float &v) const { return store.get_named(name, v); } // #<_name>, e.g. _value

    static const unsigned MAX_DEPTH = 8;
    static const unsigned MAX_ARGS = 30;
    static const unsigned MAX_NAMED = 64;
    static const unsigned MAX_SILENT_STEPS = 10000; // control steps without a line: the script loops forever

private:
    struct Frame {
        uint16_t at;      // offset of the next line to look at
        uint16_t base;    // its arguments start here in args
        bool testing;     // arrived at an elseif/else because the previous condition was false
        uint32_t has_arg;
        std::vector<std::pair<uint16_t, uint32_t> > repeats; // innermost last: repeat control -> iterations left
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
        bool set_named(const char *name, float v, std::string &err) override;
        Runner &r;
    };

    Result fail(std::string &err, const std::string &msg);
    bool control(const Control &c, const char *rest, std::string &err);
    void jump(unsigned offset) { frames.back().at = offset; }
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
    unsigned failed_at = 0;
    float abort_reason = 0;
};

}
