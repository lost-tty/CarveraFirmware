#include "Script.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

namespace script {

using gcode::skip_space;

void Source::clear()
{
    release();
    for (Segment &s : segments) free(s.path);
    segments.clear();
}

bool Source::add(const std::string &path)
{
    FILE *f = fopen(path.c_str(), "r");
    if (f == nullptr)
        return false;

    long length = fseek(f, 0, SEEK_END) == 0 ? ftell(f) : 0;
    fclose(f);
    segments.push_back(Segment{size(), uint32_t(length), strdup(path.c_str()), false});
    return true;
}

bool Source::add_job(const std::string &path)
{
    remove_job();
    if (!add(path))
        return false;

    segments.back().job = true;
    return true;
}

void Source::remove_job()
{
    int i = job();
    if (i < 0)
        return;

    release();
    free(segments[i].path);
    segments.erase(segments.begin() + i);
}

int Source::job() const
{
    for (unsigned i = 0; i < segments.size(); i++) {
        if (segments[i].job)
            return i;
    }
    return -1;
}

std::string Source::basename(unsigned segment) const
{
    const char *slash = strrchr(segments[segment].path, '/');
    return slash != nullptr ? slash + 1 : segments[segment].path;
}

std::string Source::located(const std::string &err, unsigned offset) const
{
    unsigned n = 0;
    int segment = segment_of(offset);
    if (segment < 0 || sscanf(err.c_str(), "line %u:", &n) != 1)
        return err;

    char buf[16];
    snprintf(buf, sizeof(buf), ":%u:", n);
    return basename(segment) + buf + err.substr(err.find(':') + 1);
}

uint32_t Source::mark(unsigned offset, unsigned line) const
{
    int i = segment_of(offset);
    return i < 0 ? 0 : uint32_t(i) << 24 | line;
}

std::string Source::place(uint32_t mark) const
{
    if (mark == 0 || segment(mark) >= segments.size())
        return "";

    char buf[16];
    snprintf(buf, sizeof(buf), ":%u", line(mark));
    return basename(segment(mark)) + buf;
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

bool Source::open(int segment)
{
    if (opened == segment) return true;
    release();
    fd = fopen(segments[segment].path, "r");
    if (fd == nullptr) return false;
    opened = segment;
    read_at = 0;
    return true;
}

// the only place that knows flash from file
const char *Source::chunk(unsigned offset, unsigned &length, char *buf, size_t size)
{
    int i = segment_of(offset);
    if (i < 0 || offset >= this->size()) return nullptr;
    const Segment &s = segments[i];
    unsigned avail = s.base + s.size - offset;
    if (!open(i))
        return nullptr;

    if (read_at != offset - s.base) {
        if (fseek(fd, offset - s.base, SEEK_SET) != 0)
            return nullptr;

        read_at = offset - s.base;
    }
    length = fread(buf, 1, avail < size ? avail : size, fd);
    read_at += length;
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

// label points into text
static bool o_word(const std::string &text, const char *&label, size_t &length, Kind &kind)
{
    uint8_t arg;
    std::string e;
    return is_control(text.c_str()) && split_control(text, label, length, kind, arg, e);
}

// numeric labels compare by value, so o1 and o01 are the same block
static bool label_equal(const char *a, size_t alen, const char *b, size_t blen)
{
    if (isdigit((unsigned char)a[0]) && isdigit((unsigned char)b[0])) {
        return strtol(std::string(a, alen).c_str(), nullptr, 10) == strtol(std::string(b, blen).c_str(), nullptr, 10);
    }
    return alen == blen && strncasecmp(a, b, alen) == 0;
}

// the blocks open at a line: o-words pair by label, a branch takes the place of its if or elseif
struct Blocks {
    struct Open { Kind kind; std::string label; unsigned line; uint32_t offset; };
    std::vector<Open> open;
    bool take(Kind kind, const std::string &label, unsigned line, uint32_t offset,
              std::string &err);
};

bool Blocks::take(Kind kind, const std::string &label, unsigned line, uint32_t offset,
                  std::string &err)
{
    Open *top = open.empty() ? nullptr : &open.back();
    auto ours = [&](const Open &o) {
        return label_equal(o.label.data(), o.label.size(), label.data(), label.size());
    };
    bool in_if = top != nullptr && top->kind >= IF && top->kind <= ELSE && ours(*top);
    switch (kind) {
        case SUB:
            if (top != nullptr) {
                err = std::string("sub inside ") + KEYWORDS[top->kind];
                return false;
            }
            open.push_back(Open{kind, label, line, offset});
            return true;
        case IF: case WHILE: case REPEAT:
            open.push_back(Open{kind, label, line, offset});
            return true;
        case ELSEIF: case ELSE:
            if (!in_if) {
                err = "no matching if";
                return false;
            }
            if (top->kind == ELSE) {
                err = "branch after else";
                return false;
            }
            *top = Open{kind, label, line, offset};
            return true;
        case ENDIF:
            if (!in_if) {
                err = "no matching if";
                return false;
            }
            open.pop_back();
            return true;
        case ENDWHILE: case ENDREPEAT: case ENDSUB: {
            Kind want = kind == ENDWHILE ? WHILE : kind == ENDREPEAT ? REPEAT : SUB;
            if (top == nullptr || top->kind != want || !ours(*top)) {
                err = std::string("no matching ") + KEYWORDS[want];
                return false;
            }
            open.pop_back();
            return true;
        }
        case BREAK: case CONTINUE:
            for (const Open &o : open) {
                if ((o.kind == WHILE || o.kind == REPEAT) && ours(o))
                    return true;
            }
            err = "no matching loop";
            return false;
        case RETURN: case ABORT:
            if (open.empty() || open[0].kind != SUB) {
                err = std::string(KEYWORDS[kind]) + " outside sub";
                return false;
            }
            return true;
        case CALL:
            return true;
    }
    return true;
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

bool Library::load(Source &src, std::string &err)
{
    source = &src;
    std::vector<Sub>().swap(subs);
    std::string().swap(names);
    error_offset = 0;
    Calls calls;
    for (unsigned i = 0; i < src.segments.size(); i++) {
        if (!check(i, calls, err))
            return false;
    }
    for (const std::pair<std::string, uint32_t> &c : calls) {
        if (find_sub(c.first.c_str()) < 0) {
            char buf[24];
            error_offset = c.second;
            snprintf(buf, sizeof(buf), "line %u: ", src.line_of(c.second));
            err = buf + std::string("call to unknown sub ") + c.first;
            return false;
        }
    }
    subs.shrink_to_fit();
    names.shrink_to_fit();
    src.release();
    return true;
}

bool Library::check(unsigned segment, Calls &calls, std::string &err)
{
    Source &src = *source;
    Blocks blocks;
    Permissive permissive;
    std::string text, e;
    float args[Runner::MAX_ARGS];
    unsigned end = src.segments[segment].base + src.segments[segment].size, n = 0;
    error_offset = src.segments[segment].base;

    auto unclosed = [&]() {
        char buf[48];
        error_offset = blocks.open.back().offset;
        snprintf(buf, sizeof(buf), "line %u: unclosed %s", blocks.open.back().line,
                 KEYWORDS[blocks.open.back().kind]);
        err = buf;
        return false;
    };

    for (unsigned at = src.segments[segment].base, next; at < end && src.line_at(at, text, next);
         at = next, n++) {
        trim(text);
        char lbuf[24];
        snprintf(lbuf, sizeof(lbuf), "line %u: ", n + 1);
        std::string lineno = lbuf;
        error_offset = at;
        if (text.empty() || text[0] == ';') continue;

        if (!is_control(text.c_str())) {
            gcode::Line l;
            bool ok = text[0] == '#' ? gcode::assign(text.c_str(), permissive, e) : l.parse(text.c_str(), &permissive);
            if (!ok) {
                err = lineno + (text[0] == '#' ? e : l.error_text());
                return false;
            }
            continue;
        }

        const char *label;
        size_t length;
        Kind kind;
        uint8_t arg;
        if (!split_control(text, label, length, kind, arg, e)) {
            err = lineno + e;
            return false;
        }
        if (length > MAX_LABEL) {
            err = lineno + "label too long";
            return false;
        }
        unsigned nargs;
        if (!parse_args(text.c_str() + arg, args, nargs, &permissive, e)) {
            err = lineno + e;
            return false;
        }
        bool needs_one = kind == IF || kind == ELSEIF || kind == WHILE || kind == REPEAT
                         || kind == ABORT;
        bool any = kind == RETURN || kind == CALL;
        if ((needs_one && nargs != 1) || (kind == RETURN && nargs > 1)
            || (!needs_one && !any && nargs != 0)) {
            err = lineno + (needs_one ? "expected one [condition]" : "unexpected argument");
            return false;
        }

        std::string name(label, length);
        if (kind == SUB && find_sub(name.c_str()) >= 0) {
            err = lineno + "duplicate sub " + name;
            return false;
        }
        if (!blocks.take(kind, name, n + 1, at, e)) {
            err = lineno + e;
            return false;
        }
        if (kind == SUB) {
            if (names.size() + length >= 0xFFFF) {
                err = lineno + "too many subs";
                return false;
            }
            subs.push_back(Sub{at, n + 1, uint16_t(names.size())});
            names.append(name.c_str(), name.size() + 1);
        }
        if (kind == CALL) calls.push_back(std::make_pair(name, at));
    }
    if (!blocks.open.empty())
        return unclosed(); // a block ends in the file that opened it

    return true;
}

int Library::find_sub(const char *name) const
{
    for (unsigned i = 0; i < subs.size(); i++) {
        if (label_equal(this->name(i), strlen(this->name(i)), name, strlen(name)))
            return i;
    }
    return -1;
}

int Library::find(const char *name, std::string &err)
{
    int i = find_sub(name);
    if (i >= 0)
        return i;

    std::string path;
    if (resolver != nullptr) {
        for (const std::string &p : resolver->paths(name)) {
            if (source->add(p)) {
                path = p;
                break;
            }
        }
    }
    if (path.empty()) {
        err = std::string("no sub ") + name;
        return -1;
    }
    size_t had = subs.size(), named = names.size();
    Calls calls;
    bool ok = check(source->segments.size() - 1, calls, err);
    source->release();
    if (ok && (i = find_sub(name)) >= 0)
        return i;

    err = ok ? path + " holds no o<" + name + "> sub" : source->located(err, error_offset);
    subs.resize(had);
    names.resize(named);
    free(source->segments.back().path);
    source->segments.pop_back();
    return -1;
}

void Library::reset()
{
    std::vector<Sub>().swap(subs);
    std::string().swap(names);
    if (source != nullptr)
        source->clear();
}

Runner::Runner(Library &library, gcode::ParamStore &machine)
    : library(library), machine(machine), store(*this)
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
    if (n >= 1 && n <= (int)MAX_ARGS && !r.at_main()) {
        if (r.frames.empty()) return false;
        const Frame &f = r.frames.back();
        if (!(f.has_arg & (1u << (n - 1)))) return false;
        v = r.args[f.base + n - 1];
        return true;
    }
    return r.machine.get(n, v);
}

bool Runner::Store::set(int n, float v)
{
    if (n >= 1 && n <= (int)MAX_ARGS && !r.at_main()) {
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

bool Runner::Store::has_named(const char *name) const
{
    if (name[0] == '_' && r.machine.has_named(name)) return true;
    return r.find_named(name, name[0] == '_' ? GLOBAL : r.frames.size() - 1) != nullptr;
}

bool Runner::Store::set_named(const char *name, float v, std::string &err)
{
    if (name[0] == '_') {
        if (r.machine.set_named(name, v, err)) return true;
        if (r.machine.has_named(name)) return false;
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
    f.at = sub < 0 ? 0 : library.source->next(library.subs[sub].offset);
    f.line = sub < 0 ? 1 : library.subs[sub].line + 1;
    f.base = this->args.size();
    f.testing = false;
    f.main = sub < 0;
    f.has_arg = nargs == 0 ? 0 : (nargs >= 32 ? ~0u : (1u << nargs) - 1);
    this->args.insert(this->args.end(), args, args + nargs);
    return true;
}

void Runner::pop()
{
    uint8_t depth = frames.size() - 1;
    if (frames.back().main) main_ended = Place{frames.back().at, frames.back().line};
    for (unsigned i = 0; i < named.size();) {
        if (named[i].depth == depth) named.erase(named.begin() + i);
        else i++;
    }
    args.resize(frames.back().base);
    frames.pop_back();
}

bool Runner::call(const char *sub, const float *args, unsigned nargs, std::string &err)
{
    if (frames.empty()) reset();
    if (nargs > MAX_ARGS) {
        err = "too many arguments";
        return false;
    }
    int index = library.find(sub, err);
    if (index < 0)
        return false;

    return push(index, args, nargs, err);
}

bool Runner::start_main(unsigned offset, std::string &err)
{
    reset();
    if (!push(-1, nullptr, 0, err))
        return false;

    frames[0].at = main_start = offset;
    return true;
}

bool Runner::goto_main(unsigned line, std::string &err)
{
    if (!at_main()) {
        err = "a sub is running";
        return false;
    }
    Source &src = *library.source;
    const Source::Segment &s = src.segments[src.segment_of(main_start)];
    Blocks blocks;
    std::string text, e;
    unsigned at = s.base, end = s.base + s.size, n = 1;
    for (unsigned next; n < line && at < end && src.line_at(at, text, next); at = next, n++) {
        trim(text);
        const char *label;
        size_t length;
        Kind kind;
        if (o_word(text, label, length, kind)) {
            blocks.take(kind, std::string(label, length), n, at, e);
        }
    }
    if (!blocks.open.empty()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "line %u is inside a block opened at line %u", line,
                 blocks.open.back().line);
        err = buf;
        return false;
    }
    Frame &f = frames[0];
    f.at = at;
    f.line = n;
    f.testing = false;
    f.loops.clear();
    return true;
}

bool Runner::set_local(const char *name, float v)
{
    std::string ignored;
    return !frames.empty() && name[0] != '_' && store.set_named(name, v, ignored);
}

Runner::Result Runner::fail(std::string &err, const std::string &msg)
{
    last_place = frames.empty() ? Place{0, 0} : Place{frames.back().at, frames.back().line};
    err = msg;
    stop();
    return ERROR;
}

bool Runner::arguments(const char *p, float *out, unsigned &n, std::string &err)
{
    return parse_args(p, out, n, &store, err);
}

bool Runner::call_line(const std::string &line, std::string &sub, std::string &err)
{
    std::string text = line;
    trim(text);
    const char *label;
    size_t length;
    Kind kind;
    uint8_t arg;
    if (!split_control(text, label, length, kind, arg, err))
        return false;

    if (kind != CALL) {
        err = "only a call runs from here";
        return false;
    }
    sub.assign(label, length);
    float v[MAX_ARGS];
    unsigned n;
    return arguments(text.c_str() + arg, v, n, err) && call(sub.c_str(), v, n, err);
}

bool Runner::find(const Mark &from, const char *label, size_t length, unsigned kinds, Mark &out,
                  std::string &err)
{
    Source &src = *library.source;
    int segment = src.segment_of(from.at);
    unsigned line = from.line + 1;
    Blocks blocks;
    std::string text, e;
    unsigned next;
    for (unsigned at = from.next; src.segment_of(at) == segment && src.line_at(at, text, next);
         at = next, line++) {
        trim(text);
        const char *l;
        size_t n;
        Kind kind;
        if (!o_word(text, l, n, kind))
            continue;

        if (blocks.open.empty() && (kinds & (1u << kind)) && label_equal(l, n, label, length)) {
            out = Mark{at, line, next};
            return true;
        }
        blocks.take(kind, std::string(l, n), line, at, e);
    }
    err = "no end to o" + std::string(label, length);
    return false;
}

// leaves the frame on the next line to look at
bool Runner::control(const Mark &here, const std::string &text, std::string &err)
{
    const char *label;
    size_t length;
    Kind kind;
    uint8_t arg;
    if (!split_control(text, label, length, kind, arg, err))
        return false;

    const char *rest = text.c_str() + arg;
    Frame &f = frames.back();
    float v[MAX_ARGS];
    unsigned n;
    Mark end;
    switch (kind) {
        case SUB: // a definition met in straight-line flow: skip over it
            if (!find(here, label, length, 1u << ENDSUB, end, err))
                return false;

            jump_after(end);
            return true;
        case ENDSUB: case RETURN:
            if (f.main) {
                err = std::string(KEYWORDS[kind]) + " outside sub";
                return false;
            }
            n = 0;
            if (kind == RETURN && !arguments(rest, v, n, err))
                return false;

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
            unsigned caller = frames.size() - 1;
            std::string sub;
            if (!call_line(text, sub, err))
                return false;

            frames[caller].at = here.next; // by index: push may reallocate
            frames[caller].line = here.line + 1;
            return true;
        }
        case IF: case ELSEIF: case ELSE: {
            if (kind != IF && !f.testing) { // fell out of a taken branch: to the endif
                if (!find(here, label, length, 1u << ENDIF, end, err))
                    return false;

                jump_after(end);
                return true;
            }
            bool cond = kind == ELSE;
            if (kind != ELSE) {
                if (!arguments(rest, v, n, err)) return false;
                cond = v[0] != 0;
            }
            f.testing = !cond;
            if (cond) {
                jump_after(here);
                return true;
            }
            if (!find(here, label, length, 1u << ELSEIF | 1u << ELSE | 1u << ENDIF, end, err)) {
                return false;
            }
            jump_to(end.at, end.line);
            return true;
        }
        case ENDIF:
            f.testing = false;
            jump_after(here);
            return true;
        case WHILE: case REPEAT: {
            bool entered = !f.loops.empty() && f.loops.back().at == here.at;
            if (!entered || kind == WHILE) {
                if (!arguments(rest, v, n, err)) return false;
                if (kind == REPEAT && v[0] < 0) {
                    err = "negative repeat count";
                    return false;
                }
            }
            if (!entered) {
                uint32_t left = kind == REPEAT ? uint32_t(v[0]) : 0;
                f.loops.push_back(Loop{here.at, here.line, left, std::string(label, length)});
            }
            bool more = kind == WHILE ? v[0] != 0 : f.loops.back().left > 0;
            if (more) {
                if (kind == REPEAT) f.loops.back().left--;
                jump_after(here);
                return true;
            }
            f.loops.pop_back();
            unsigned ends = kind == WHILE ? 1u << ENDWHILE : 1u << ENDREPEAT;
            if (!find(here, label, length, ends, end, err))
                return false;

            jump_after(end);
            return true;
        }
        case ENDWHILE: case ENDREPEAT: case CONTINUE: case BREAK: {
            // loops opened inside this one sit above it on the stack
            auto ours = [&]() {
                const std::string &l = f.loops.back().label;
                return label_equal(l.data(), l.size(), label, length);
            };
            bool leaving = kind == BREAK || kind == CONTINUE;
            while (leaving && !f.loops.empty() && !ours()) f.loops.pop_back();
            if (f.loops.empty() || !ours()) {
                err = std::string(KEYWORDS[kind]) + " outside its loop";
                return false;
            }
            if (kind != BREAK) {
                jump_to(f.loops.back().at, f.loops.back().line);
                return true;
            }
            f.loops.pop_back();
            if (!find(here, label, length, 1u << ENDWHILE | 1u << ENDREPEAT, end, err)) {
                return false;
            }
            jump_after(end);
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

Runner::Result Runner::step(std::string &out, std::string &err)
{
    std::string text;
    while (!frames.empty()) {
        Frame &f = frames.back();
        unsigned at = f.at, next;
        if (!library.source->line_at(at, text, next)) {
            pop();
            if (at_main())
                return RETURNED;

            continue;
        }
        trim(text);

        std::string e;
        bool msg = text.size() > 5 && text[0] == '(' && (strncasecmp(text.c_str() + 1, "MSG,", 4) == 0 || strncasecmp(text.c_str() + 1, "DEBUG,", 6) == 0 || strncasecmp(text.c_str() + 1, "PRINT,", 6) == 0);
        if (msg) {
            if (!message(text, out, e)) return fail(err, e);
            took(f, at, next);
            silent_steps = 0;
            return MESSAGE;
        }
        if (text.empty() || text[0] == ';') {
            advance(f, next);
            continue;
        }
        bool ctl = is_control(text.c_str());
        if ((ctl || text[0] == '#') && ++silent_steps > MAX_SILENT_STEPS) {
            return fail(err, "script does not progress");
        }
        if (ctl) {
            size_t depth = frames.size();
            if (!control(Mark{at, f.line, next}, text, e))
                return fail(err, e);

            if (frames.size() < depth && at_main())
                return RETURNED;
        } else if (text[0] == '#') {
            if (!gcode::assign(text.c_str(), store, e))
                return fail(err, e);

            advance(f, next);
        } else {
            took(f, at, next);
            gcode::Line words;
            if (words.parse(text.c_str(), nullptr) && words.words().empty())
                continue; // a comment

            out = text;
            silent_steps = 0;
            return LINE;
        }
    }
    return DONE;
}

}
