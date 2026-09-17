#include "Script.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <strings.h>

namespace script {

using gcode::skip_space;

Source::Source(const char *text) : Source(text, strlen(text)) {}

Source::Source(const char *text, size_t length)
{
    add(text, length, "", 0);
}

void Source::clear()
{
    release();
    for (Segment &s : segments) if (s.owned) delete[] const_cast<char *>(s.name);
    segments.clear();
}

bool Source::add(const char *flash, size_t length, const char *name, uint8_t name_length)
{
    if (size() + length > 0xFFFF) return false;
    segments.push_back(Segment{uint16_t(size()), uint16_t(length), flash, name, name_length, 0});
    return true;
}

// the name is copied: it comes from a directory listing that does not outlive the load
bool Source::add_file(const char *d, const std::string &name, size_t length)
{
    if (name.size() > 255 || size() + length > 0xFFFF) return false;
    char *copy = new char[name.size()];
    memcpy(copy, name.data(), name.size());
    dir = d;
    segments.push_back(Segment{uint16_t(size()), uint16_t(length), nullptr, copy, uint8_t(name.size()), 1});
    return true;
}

void Source::release()
{
    if (fd != nullptr) fclose(fd);
    fd = nullptr;
    opened = -1;
}

int Source::segment_of(unsigned offset) const
{
    for (int i = segments.size() - 1; i >= 0; i--) if (offset >= segments[i].base) return i;
    return -1;
}

unsigned Source::opens = 0; // host tests check that flash-only loading touches no files

bool Source::open(int segment)
{
    if (opened == segment) return true;
    opens++;
    release();
    std::string path = std::string(dir) + name(segment);
    fd = fopen(path.c_str(), "r");
    if (fd == nullptr) return false;
    opened = segment;
    return true;
}

// the only place that knows flash from file
const char *Source::chunk(unsigned offset, unsigned &length, char *buf, size_t size)
{
    int i = segment_of(offset);
    if (i < 0 || offset >= this->size()) return nullptr;
    const Segment &s = segments[i];
    unsigned avail = s.base + s.size - offset;
    if (s.flash != nullptr) {
        length = avail;
        return s.flash + (offset - s.base);
    }
    if (!open(i) || fseek(fd, offset - s.base, SEEK_SET) != 0) return nullptr;
    length = fread(buf, 1, avail < size ? avail : size, fd);
    return length > 0 ? buf : nullptr;
}

bool Source::read(unsigned offset, char *out, size_t length)
{
    char buf[64];
    if (length > sizeof(buf)) return false;
    unsigned avail;
    const char *p = chunk(offset, avail, buf, sizeof(buf));
    if (p == nullptr || avail < length) return false;
    memcpy(out, p, length);
    return true;
}

// the line at offset without its newline; next is the following line, which may start the next segment
bool Source::line_at(unsigned offset, std::string &out, unsigned &next)
{
    char buf[MAX_LINE];
    out.clear();
    for (unsigned at = offset;;) {
        unsigned avail;
        const char *p = chunk(at, avail, buf, sizeof(buf));
        if (p == nullptr) {
            next = at;
            return at > offset; // the last line of the source has no newline
        }
        const char *e = (const char *)memchr(p, '\n', avail);
        unsigned len = e ? unsigned(e - p) : avail;
        out.append(p, len);
        at += len + (e != nullptr);
        if (e != nullptr || segment_of(at) != segment_of(offset)) { // a segment end also ends the line
            next = at;
            return true;
        }
    }
}

unsigned Source::next(unsigned offset)
{
    std::string ignored;
    unsigned n;
    return line_at(offset, ignored, n) ? n : size();
}

unsigned Source::line_of(unsigned offset)
{
    int i = segment_of(offset);
    if (i < 0) return 0;
    unsigned n = 1;
    char buf[MAX_LINE];
    for (unsigned at = segments[i].base; at < offset; ) {
        unsigned avail;
        const char *p = chunk(at, avail, buf, sizeof(buf));
        if (p == nullptr) break;
        if (avail > offset - at) avail = offset - at;
        for (const char *e = p; (e = (const char *)memchr(e, '\n', p + avail - e)) != nullptr; e++) n++;
        at += avail;
    }
    return n;
}

// returns how many leading characters went
static unsigned trim(std::string &s)
{
    size_t b = s.find_first_not_of(" \t\r");
    size_t e = s.find_last_not_of(" \t\r");
    s = b == std::string::npos ? "" : s.substr(b, e - b + 1);
    return b == std::string::npos ? 0 : b;
}

static bool is_control(const std::string &s)
{
    return s.size() > 1 && (s[0] == 'o' || s[0] == 'O') && (s[1] == '<' || isdigit((unsigned char)s[1]));
}

static const char *KEYWORDS[] = {"sub", "endsub", "call", "return", "abort", "if", "elseif", "else", "endif",
                                 "while", "endwhile", "break", "continue", "repeat", "endrepeat"};

// "o<name> keyword rest": label = the name or number as written, arg = offset of rest
static bool split_control(const std::string &s, const char *&label, size_t &length, Kind &kind, uint8_t &arg, std::string &err)
{
    const char *p = s.c_str() + 1;
    if (*p == '<') {
        std::string name;
        if (!gcode::named_param(p, name, err)) return false; // validates; p is past the >
        label = s.c_str() + 2;
        length = p - label - 1;
    } else {
        label = p;
        strtol(p, const_cast<char **>(&p), 10);
        length = p - label;
    }
    skip_space(p);
    const char *k = p;
    while (isalpha((unsigned char)*p)) p++;
    for (unsigned n = 0; n < sizeof(KEYWORDS) / sizeof(*KEYWORDS); n++) {
        if (strlen(KEYWORDS[n]) == size_t(p - k) && strncasecmp(k, KEYWORDS[n], p - k) == 0) {
            kind = Kind(n);
            skip_space(p);
            arg = p - s.c_str();
            return true;
        }
    }
    err = "unknown o-word " + std::string(k, p - k);
    return false;
}

// numeric labels compare by value, so o1 and o01 are the same block
static bool label_equal(const char *a, size_t alen, const char *b, size_t blen)
{
    if (isdigit((unsigned char)a[0]) && isdigit((unsigned char)b[0])) {
        return strtol(std::string(a, alen).c_str(), nullptr, 10) == strtol(std::string(b, blen).c_str(), nullptr, 10);
    }
    return alen == blen && strncasecmp(a, b, alen) == 0;
}

// [expr] [expr] ... up to MAX_ARGS values; a trailing comment is allowed
static bool parse_args(const char *p, float *out, unsigned &n, const gcode::ParamStore *params, std::string &err)
{
    n = 0;
    for (;;) {
        skip_space(p);
        if (*p == 0 || *p == ';' || *p == '(') return true;
        if (n == Runner::MAX_ARGS) {
            err = "too many arguments";
            return false;
        }
        if (*p != '[') {
            err = "expected [expression]";
            return false;
        }
        if (!gcode::eval(p, out[n++], params, err)) return false;
    }
}

// tokenizes with every parameter defined, to find syntax errors before anything runs
class Permissive : public gcode::ParamStore {
public:
    bool get(int, float &v) const override { v = 0; return true; }
    bool set(int, float) override { return true; }
    bool get_named(const char *, float &v) const override { v = 0; return true; }
    bool set_named(const char *, float, std::string &) override { return true; }
};

bool Program::label_is(const Label &l, const char *name, size_t length) const
{
    char buf[MAX_LABEL];
    if (!source->read(l.offset, buf, l.length)) return false;
    return label_equal(buf, l.length, name, length);
}

std::string Program::label_text(const Label &l) const
{
    char buf[MAX_LABEL];
    if (!source->read(l.offset, buf, l.length)) return "?";
    return std::string(buf, l.length);
}

bool Program::load(Source &src, std::string &err)
{
    source = &src;
    controls.clear();
    labels.clear();
    error_offset = 0;
    unsigned count = 0; // o-word lines, so the table is allocated once at its final size
    std::string text;
    for (unsigned at = 0, next; src.line_at(at, text, next); at = next) {
        size_t b = text.find_first_not_of(" \t");
        if (b != std::string::npos && b + 1 < text.size() && (text[b] == 'o' || text[b] == 'O') && (text[b + 1] == '<' || isdigit((unsigned char)text[b + 1]))) count++;
    }
    controls.reserve(count);

    Permissive permissive;
    std::vector<uint16_t> open; // unclosed sub/if(latest branch)/while/repeat
    std::string e;
    float args[Runner::MAX_ARGS];

    int segment = -1;
    unsigned n = 0;
    for (unsigned at = 0, next; src.line_at(at, text, next); at = next, n++) {
        if (src.segment_of(at) != segment) { // line numbers restart with each file
            segment = src.segment_of(at);
            n = 0;
        }
        unsigned leading = trim(text);
        char lbuf[24];
        snprintf(lbuf, sizeof(lbuf), "line %u: ", n + 1);
        std::string lineno = lbuf;
        error_offset = at;
        if (text.empty() || text[0] == ';') continue;

        if (!is_control(text)) {
            gcode::Line l;
            bool ok = text[0] == '#' ? gcode::assign(text.c_str(), permissive, e) : l.parse(text.c_str(), &permissive);
            if (!ok) {
                err = lineno + (text[0] == '#' ? e : l.error_text());
                return false;
            }
            continue;
        }

        Control c{uint16_t(at), SUB, 0, 0, 0};
        const char *label;
        size_t length;
        if (!split_control(text, label, length, c.kind, c.arg, e)) {
            err = lineno + e;
            return false;
        }
        if (length > MAX_LABEL) {
            err = lineno + "label too long";
            return false;
        }
        c.label = labels.size();
        for (uint16_t i = 0; i < labels.size(); i++) if (label_is(labels[i], label, length)) c.label = i;
        if (c.label == labels.size()) labels.push_back(Label{uint16_t(at + leading + (label - text.c_str())), uint16_t(length)});

        unsigned nargs;
        if (!parse_args(text.c_str() + c.arg, args, nargs, &permissive, e)) {
            err = lineno + e;
            return false;
        }
        bool needs_one = c.kind == IF || c.kind == ELSEIF || c.kind == WHILE || c.kind == REPEAT || c.kind == ABORT;
        if ((needs_one && nargs != 1) || (c.kind == RETURN && nargs > 1) || (!needs_one && c.kind != RETURN && c.kind != CALL && nargs != 0)) {
            err = lineno + (needs_one ? "expected one [condition]" : "unexpected argument");
            return false;
        }

        Control *top = open.empty() ? nullptr : &controls[open.back()];
        bool in_sub = !open.empty() && controls[open[0]].kind == SUB;
        uint16_t index = controls.size();
        switch (c.kind) {
            case SUB:
                if (top != nullptr) { err = lineno + "sub inside " + KEYWORDS[top->kind]; return false; }
                if (find_sub(std::string(label, length).c_str()) >= 0) { err = lineno + "duplicate sub " + std::string(label, length); return false; }
                open.push_back(index);
                break;
            case IF: case WHILE: case REPEAT:
                open.push_back(index);
                break;
            case ELSEIF: case ELSE:
                if (top == nullptr || top->kind < IF || top->kind > ELSE || top->label != c.label) { err = lineno + "no matching if"; return false; }
                if (top->kind == ELSE) { err = lineno + "branch after else"; return false; }
                top->match = index;
                open.back() = index; // the chain continues from this branch
                break;
            case ENDIF:
                if (top == nullptr || top->kind < IF || top->kind > ELSE || top->label != c.label) { err = lineno + "no matching if"; return false; }
                top->match = index;
                open.pop_back();
                break;
            case ENDWHILE: case ENDREPEAT: case ENDSUB: {
                Kind want = c.kind == ENDWHILE ? WHILE : c.kind == ENDREPEAT ? REPEAT : SUB;
                if (top == nullptr || top->kind != want || top->label != c.label) { err = lineno + "no matching " + KEYWORDS[want]; return false; }
                top->match = index;
                c.match = open.back();
                open.pop_back();
                break;
            }
            case BREAK: case CONTINUE: {
                int loop = -1;
                for (int i = open.size() - 1; i >= 0 && loop < 0; i--) {
                    const Control &o = controls[open[i]];
                    if ((o.kind == WHILE || o.kind == REPEAT) && o.label == c.label) loop = open[i];
                }
                if (loop < 0) { err = lineno + "no matching loop"; return false; }
                c.match = loop;
                break;
            }
            case RETURN: case ABORT:
                if (!in_sub) { err = lineno + std::string(KEYWORDS[c.kind]) + " outside sub"; return false; }
                break;
            case CALL:
                break;
        }
        if (index >= 1000) { err = lineno + "too many o-words"; return false; }
        controls.push_back(c);
    }
    if (!open.empty()) {
        char buf[48];
        error_offset = controls[open.back()].offset;
        snprintf(buf, sizeof(buf), "line %u: unclosed %s", src.line_of(error_offset), KEYWORDS[controls[open.back()].kind]);
        err = buf;
        return false;
    }
    for (Control &c : controls) {
        if (c.kind != CALL) continue;
        int sub = find_sub(label_text(labels[c.label]).c_str());
        if (sub < 0) {
            char buf[24];
            error_offset = c.offset;
            snprintf(buf, sizeof(buf), "line %u: ", src.line_of(c.offset));
            err = buf + std::string("call to unknown sub ") + label_text(labels[c.label]);
            return false;
        }
        c.match = sub;
    }
    labels.shrink_to_fit();
    src.release();
    return true;
}

int Program::find_sub(const char *name) const
{
    for (unsigned i = 0; i < controls.size(); i++) {
        if (controls[i].kind == SUB && label_is(labels[controls[i].label], name, strlen(name))) return i;
    }
    return -1;
}

const Control *Program::control_at(unsigned offset) const
{
    unsigned lo = 0, hi = controls.size();
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (controls[mid].offset < offset) lo = mid + 1;
        else hi = mid;
    }
    return lo < controls.size() && controls[lo].offset == offset ? &controls[lo] : nullptr;
}

Runner::Runner(const Program &program, gcode::ParamStore &machine) : program(program), machine(machine), store(*this)
{
    frames.reserve(MAX_DEPTH);
    named.push_back(Named{GLOBAL, "_value", 0});
    named.push_back(Named{GLOBAL, "_value_returned", 0});
}

Runner::Named *Runner::find_named(const char *name, uint8_t depth)
{
    for (Named &n : named) if (n.depth == depth && n.name == name) return &n;
    return nullptr;
}

bool Runner::Store::get(int n, float &v) const
{
    if (n >= 1 && n <= (int)MAX_ARGS) {
        const Frame &f = r.frames.back();
        if (!(f.has_arg & (1u << (n - 1)))) return false;
        v = r.args[f.base + n - 1];
        return true;
    }
    return r.machine.get(n, v);
}

bool Runner::Store::set(int n, float v)
{
    if (n >= 1 && n <= (int)MAX_ARGS) {
        Frame &f = r.frames.back();
        if (r.args.size() < size_t(f.base + n)) r.args.resize(f.base + n, 0);
        r.args[f.base + n - 1] = v;
        f.has_arg |= 1u << (n - 1);
        return true;
    }
    return r.machine.set(n, v);
}

bool Runner::Store::get_named(const char *name, float &v) const
{
    if (name[0] == '_' && r.machine.get_named(name, v)) return true;
    const Named *n = r.find_named(name, name[0] == '_' ? GLOBAL : r.frames.size() - 1);
    if (n == nullptr) return false;
    v = n->value;
    return true;
}

bool Runner::Store::set_named(const char *name, float v, std::string &err)
{
    if (name[0] == '_') {
        if (r.machine.set_named(name, v, err)) return true;
        float ignored;
        if (r.machine.get_named(name, ignored)) return false; // the machine owns it: not a script global
    }
    uint8_t depth = name[0] == '_' ? GLOBAL : r.frames.size() - 1;
    if (Named *n = r.find_named(name, depth)) {
        n->value = v;
        return true;
    }
    if (r.named.size() >= MAX_NAMED) {
        err = "too many named parameters";
        return false;
    }
    r.named.push_back(Named{depth, name, v});
    return true;
}

bool Runner::push(int sub, const float *args, unsigned nargs, std::string &err)
{
    if (frames.size() >= MAX_DEPTH) {
        err = "call too deep";
        return false;
    }
    frames.emplace_back();
    Frame &f = frames.back();
    f.at = sub < 0 ? 0 : program.after(program.controls[sub]);
    f.base = this->args.size();
    f.testing = false;
    f.has_arg = nargs == 0 ? 0 : (nargs >= 32 ? ~0u : (1u << nargs) - 1);
    this->args.insert(this->args.end(), args, args + nargs);
    return true;
}

void Runner::pop()
{
    uint8_t depth = frames.size() - 1;
    for (unsigned i = 0; i < named.size();) {
        if (named[i].depth == depth) named.erase(named.begin() + i);
        else i++;
    }
    args.resize(frames.back().base);
    frames.pop_back();
}

bool Runner::start(const char *sub, const float *args, unsigned nargs, std::string &err)
{
    stop();
    silent_steps = 0;
    abort_reason = 0;
    int index = -1;
    if (sub != nullptr && (index = program.find_sub(sub)) < 0) {
        err = std::string("no sub ") + sub;
        return false;
    }
    if (nargs > MAX_ARGS) {
        err = "too many arguments";
        return false;
    }
    return push(index, args, nargs, err);
}

bool Runner::set_local(const char *name, float v)
{
    std::string ignored;
    return !frames.empty() && name[0] != '_' && store.set_named(name, v, ignored);
}

Runner::Result Runner::fail(std::string &err, const std::string &msg)
{
    char buf[16];
    failed_at = frames.empty() ? 0 : frames.back().at;
    snprintf(buf, sizeof(buf), "line %u: ", frames.empty() ? 0 : program.source->line_of(failed_at));
    err = buf + msg;
    stop();
    return ERROR;
}

bool Runner::arguments(const char *p, float *out, unsigned &n, std::string &err)
{
    return parse_args(p, out, n, &store, err);
}

// leaves the frame on the next line to look at
bool Runner::control(const Control &c, const char *rest, std::string &err)
{
    Frame &f = frames.back();
    float v[MAX_ARGS];
    unsigned n;
    const std::vector<Control> &cs = program.controls;
    switch (c.kind) {
        case SUB: // a definition met in straight-line flow: skip over it
            jump(program.after(cs[c.match]));
            return true;
        case ENDSUB: case RETURN:
            n = 0;
            if (c.kind == RETURN && !arguments(rest, v, n, err)) return false;
            if (n) find_named("_value", GLOBAL)->value = v[0];
            find_named("_value_returned", GLOBAL)->value = n;
            pop();
            return true;
        case ABORT:
            if (!arguments(rest, v, n, err)) return false;
            abort_reason = v[0] != 0 ? v[0] : 1;
            stop();
            return true;
        case CALL: {
            if (!arguments(rest, v, n, err)) return false;
            unsigned caller = frames.size() - 1;
            if (!push(c.match, v, n, err)) return false;
            frames[caller].at = program.after(c); // by index: push may reallocate
            return true;
        }
        case IF: case ELSEIF: case ELSE: {
            if (c.kind != IF && !f.testing) { // fell out of a taken branch: to the endif
                const Control *b = &c;
                while (b->kind != ENDIF) b = &cs[b->match];
                jump(program.after(*b));
                return true;
            }
            bool cond = c.kind == ELSE;
            if (c.kind != ELSE) {
                if (!arguments(rest, v, n, err)) return false;
                cond = v[0] != 0;
            }
            f.testing = !cond;
            jump(cond ? program.after(c) : cs[c.match].offset);
            return true;
        }
        case ENDIF:
            f.testing = false;
            jump(program.after(c));
            return true;
        case WHILE:
            if (!arguments(rest, v, n, err)) return false;
            jump(v[0] != 0 ? program.after(c) : program.after(cs[c.match]));
            return true;
        case ENDWHILE:
            jump(cs[c.match].offset); // re-test
            return true;
        case REPEAT: {
            uint16_t index = &c - &cs[0];
            if (f.repeats.empty() || f.repeats.back().first != index) {
                if (!arguments(rest, v, n, err)) return false;
                if (v[0] < 0) {
                    err = "negative repeat count";
                    return false;
                }
                f.repeats.push_back(std::make_pair(index, uint32_t(v[0])));
            }
            if (f.repeats.back().second == 0) {
                f.repeats.pop_back();
                jump(program.after(cs[c.match]));
            } else {
                f.repeats.back().second--;
                jump(program.after(c));
            }
            return true;
        }
        case ENDREPEAT:
            jump(cs[c.match].offset); // count down
            return true;
        case BREAK: case CONTINUE: {
            // loops nest in index order, so everything opened inside the target loop is above it on the stack
            while (!f.repeats.empty() && (f.repeats.back().first > c.match || (c.kind == BREAK && f.repeats.back().first == c.match))) {
                f.repeats.pop_back();
            }
            const Control &loop = cs[c.match];
            jump(c.kind == BREAK ? program.after(cs[loop.match]) : cs[loop.match].offset);
            return true;
        }
    }
    return false;
}

// %.4f without trailing zeros; false when it does not fit
static bool format_value(char *buf, size_t size, float v)
{
    int n = snprintf(buf, size, "%.4f", v);
    if (n >= (int)size) return false;
    for (char *e = buf + n - 1; *e == '0' || *e == '.'; e--) {
        bool dot = *e == '.';
        *e = 0;
        if (dot) break;
    }
    return true;
}

// (MSG, text) as is; (DEBUG, text) and (PRINT, text) with #n and #<name> replaced by their values
bool Runner::message(const std::string &text, std::string &out, std::string &err)
{
    size_t comma = text.find(','), close = text.rfind(')');
    if (close == std::string::npos || close < comma) close = text.size(); // an unclosed comment runs to the end
    bool expand = strncasecmp(text.c_str(), "(MSG", 4) != 0;
    out.clear();
    for (size_t i = comma + 1; i < close; i++) {
        if (text[i] == ' ' && out.empty()) continue;
        if (text[i] != '#' || !expand) {
            out += text[i];
            continue;
        }
        const char *p = text.c_str() + i + 1;
        float v;
        std::string name;
        bool ok;
        if (*p == '<') {
            ok = gcode::named_param(p, name, err) && store.get_named(name.c_str(), v);
            if (!ok && err.empty()) err = "no value for parameter #<" + name + ">";
        } else {
            char *end;
            long n = strtol(p, &end, 10);
            ok = end != p && store.get(n, v);
            if (!ok) err = end == p ? "bad parameter number" : "no value for parameter #" + std::string(p, end - p);
            p = end;
        }
        if (!ok) return false;
        char buf[24];
        if (!format_value(buf, sizeof(buf), v)) {
            err = "value out of range";
            return false;
        }
        out += buf;
        i = p - text.c_str() - 1;
    }
    return true;
}

bool Runner::substitute(const std::string &text, std::string &out, std::string &err)
{
    if (text[0] != '(' && text[0] != '%' && text.find('#') == std::string::npos && text.find('[') == std::string::npos) {
        out = text;
        return true;
    }
    gcode::Line l;
    if (!l.parse(text.c_str(), &store)) {
        err = l.error_text();
        return false;
    }
    out.clear();
    char buf[24];
    for (const gcode::Word &w : l.words()) {
        int n;
        if (w.letter == 'G' || w.letter == 'M') {
            n = snprintf(buf, sizeof(buf), w.subcode ? "%c%d.%d" : "%c%d", w.letter, (int)w.value, w.subcode);
        } else if (!w.has_value) {
            n = snprintf(buf, sizeof(buf), "%c", w.letter);
        } else {
            buf[0] = w.letter;
            n = format_value(buf + 1, sizeof(buf) - 1, w.value) ? 1 : (int)sizeof(buf);
        }
        if (n >= (int)sizeof(buf)) {
            err = "value out of range";
            return false;
        }
        if (!out.empty()) out += ' ';
        out += buf;
    }
    return true;
}

Runner::Result Runner::step(std::string &out, std::string &err)
{
    std::string text;
    while (!frames.empty()) {
        Frame &f = frames.back();
        unsigned at = f.at, next;
        if (!program.source->line_at(at, text, next)) {
            pop();
            continue;
        }
        trim(text);
        if (++silent_steps > MAX_SILENT_STEPS) return fail(err, "script does not progress");

        std::string e;
        bool msg = text.size() > 5 && text[0] == '(' && (strncasecmp(text.c_str() + 1, "MSG,", 4) == 0 || strncasecmp(text.c_str() + 1, "DEBUG,", 6) == 0 || strncasecmp(text.c_str() + 1, "PRINT,", 6) == 0);
        if (msg) {
            if (!message(text, out, e)) return fail(err, e);
            current = at;
            f.at = next;
            silent_steps = 0;
            return MESSAGE;
        }
        if (text.empty() || text[0] == ';') {
            f.at = next;
        } else if (const Control *c = program.control_at(at)) {
            if (!control(*c, text.c_str() + c->arg, e)) return fail(err, e);
        } else if (text[0] == '#') {
            if (!gcode::assign(text.c_str(), store, e)) return fail(err, e);
            f.at = next;
        } else {
            if (!substitute(text, out, e)) return fail(err, e);
            current = at;
            f.at = next;
            if (out.empty()) continue; // a comment
            silent_steps = 0;
            return LINE;
        }
    }
    return DONE;
}

}
