// Host test of the script loader: names, reading a sub on its first call, SD replacements, line
// mapping, memory use.
// usage: macros_test <directory standing for /macros, e.g. ../src/macros> <scratch directory>
#include "Macros.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <new>
#include <sstream>
#include <sys/stat.h>

static size_t live = 0, peak = 0;
void *operator new(size_t n) {
    size_t *p = (size_t *)malloc(n + 16);
    if (!p) throw std::bad_alloc();
    *p = n; live += n; if (live > peak) peak = live;
    return (char *)p + 16;
}
void operator delete(void *q) noexcept { if (!q) return; size_t *p = (size_t *)((char *)q - 16); live -= *p; free(p); }
void operator delete(void *q, size_t) noexcept { operator delete(q); }

static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

static void write(const std::string &path, const std::string &content) { std::ofstream(path) << content; }

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    std::string embedded = std::string(argv[1]) + "/";
    unsigned files = 0;
    if (DIR *d = opendir(embedded.c_str())) {
        while (struct dirent *e = readdir(d)) files += strstr(e->d_name, ".ngc") != nullptr;
        closedir(d);
    }
    std::string dir = std::string(argv[2]) + "/macros/";
    mkdir(dir.c_str(), 0755);

    { // the names only, nothing read
        size_t before = live;
        Macros m;
        m.load({embedded});
        CHECK(m.names().size() == files);
        CHECK(m.has("m6") && m.has("M6") && m.has("atc_change") && !m.has("m7"));
        CHECK(m.paths("m6") == std::vector<std::string>{embedded + "m6.ngc"});
        CHECK(m.paths("m7").empty());
        printf("names: %zu bytes live for %u files\n", live - before, files);
    }
    { // a call reads the file once, reset forgets it
        Macros m; std::string err;
        script::Source src; script::Library prog;
        prog.source = &src; prog.resolver = &m;
        m.load({embedded});
        size_t before = live;
        int sub = prog.find("m6", err);
        CHECK(sub >= 0 && err.empty());
        CHECK(src.segments.size() == 1
              && src.basename(src.segment_of(prog.subs[sub].offset)) == "m6.ngc");
        CHECK(prog.find("atc_change", err) >= 0 && src.segments.size() == 2);
        CHECK(prog.find("m6", err) == sub && src.segments.size() == 2);
        printf("two subs read: %zu bytes live\n", live - before);
        prog.reset();
        CHECK(src.segments.empty() && prog.subs.empty());
        CHECK(prog.find("nothing", err) < 0 && err == "no sub nothing");
    }
    { // an SD file replaces the embedded one and adds one
        write(dir + "g28.ngc", "o<g28> sub\n(MSG, custom)\no<g28> endsub\n");
        write(dir + "extra.ngc", "(added)\no<extra> sub\nG0 X0\no<extra> endsub\n");
        write(dir + "notes.txt", "ignored");
        Macros m; std::string err;
        script::Source src; script::Library prog;
        prog.source = &src; prog.resolver = &m;
        m.load({dir, embedded});
        CHECK(m.names().size() == files + 1);
        CHECK(m.paths("g28") == (std::vector<std::string>{dir + "g28.ngc", embedded + "g28.ngc"}));
        int extra = prog.find("extra", err);
        CHECK(extra >= 0);
        unsigned at = prog.subs[extra].offset;
        CHECK(src.basename(src.segment_of(at)) == "extra.ngc" && src.line_of(at) == 2);
        CHECK(src.located("line 2: x", at) == "extra.ngc:2: x");
        std::string line; unsigned next;
        int g28 = prog.find("g28", err);
        CHECK(g28 >= 0 && src.line_at(prog.subs[g28].offset + 11, line, next)
              && line == "(MSG, custom)");
        src.release();
    }
    { // a broken file fails its call with the place, and leaves nothing behind
        write(dir + "broken.ngc", "o<broken> sub\nG0 X0\n");
        write(dir + "other.ngc", "o<elsewhere> sub\no<elsewhere> endsub\n");
        Macros m; std::string err;
        script::Source src; script::Library prog;
        prog.source = &src; prog.resolver = &m;
        m.load({dir, embedded});
        CHECK(prog.find("broken", err) < 0 && err.compare(0, 13, "broken.ngc:1:") == 0);
        CHECK(src.segments.empty() && prog.subs.empty());
        CHECK(prog.find("other", err) < 0
              && err.find("holds no o<other> sub") != std::string::npos);
        CHECK(prog.find("m6", err) >= 0);
        remove((dir + "broken.ngc").c_str());
        remove((dir + "other.ngc").c_str());
    }
    { // past 64 KB
        write(dir + "huge.ngc", std::string(70000, '\n') + "o<huge> sub\nG0 X1\no<huge> endsub\n");
        Macros m; std::string err;
        script::Source src; script::Library prog;
        prog.source = &src; prog.resolver = &m;
        m.load({dir, embedded});
        int huge = prog.find("huge", err);
        CHECK(huge >= 0 && prog.subs[huge].line == 70001);
        remove((dir + "huge.ngc").c_str());
    }
    remove((dir + "g28.ngc").c_str()); remove((dir + "extra.ngc").c_str()); remove((dir + "notes.txt").c_str());
    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
