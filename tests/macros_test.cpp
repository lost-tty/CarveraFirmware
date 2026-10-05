// Host test of the script loader: the embedded files, SD replacements and additions, line mapping,
// memory use.
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

    { // embedded only
        size_t before = live, peak_before = peak = live;
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(embedded.c_str(), nullptr, r, err));
        CHECK(err.empty());
        CHECK(r.embedded == files && r.replaced == 0 && r.added == 0 && r.fallback.empty());
        CHECK(m.program().find_sub("m6") >= 0 && m.program().find_sub("atc_change") >= 0);
        int sub = m.program().find_sub("atc_change");
        unsigned at = m.program().subs[sub].offset;
        CHECK(m.file(at) == "atc_change.ngc");                       // the first embedded file, alphabetically
        CHECK(m.source().line_of(at) == 2);
        CHECK(m.located("line 2: bad", at) == "atc_change.ngc:2: bad");
        CHECK(m.located("no line", at) == "no line");
        CHECK(m.source().segments.size() == files);
        CHECK(std::string(m.source().segments[0].path) == embedded + "atc_change.ngc");
        CHECK(m.source().line_of(m.source().segments[1].base) == 1);
        m.source().release();
        printf("embedded load: %zu bytes live, %zu peak\n", live - before, peak - peak_before);
    }
    { // SD replaces one file and adds one
        write(dir + "g28.ngc", "o<g28> sub\n(MSG, custom)\no<g28> endsub\n");
        write(dir + "extra.ngc", "(added)\no<extra> sub\nG0 X0\no<extra> endsub\n");
        write(dir + "notes.txt", "ignored");
        size_t before = live;
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(embedded.c_str(), dir.c_str(), r, err));
        CHECK(r.embedded == files && r.replaced == 1 && r.added == 1 && r.fallback.empty());
        int extra = m.program().find_sub("extra");
        CHECK(extra >= 0);
        unsigned at = m.program().subs[extra].offset;
        CHECK(m.file(at) == "extra.ngc" && m.source().line_of(at) == 2);
        char buf[32]; snprintf(buf, sizeof(buf), "line %u: x", m.source().line_of(at));
        CHECK(m.located(buf, at) == "extra.ngc:2: x");
        std::string line; unsigned next;
        CHECK(m.source().line_at(m.program().subs[m.program().find_sub("g28")].offset + 11, line, next) && line == "(MSG, custom)");
        m.source().release();
        printf("sd load: %zu bytes live (%zu segments, %zu subs)\n", live - before,
               m.source().segments.size(), m.program().subs.size());
    }
    { // a broken SD file drops the whole SD set, the embedded scripts stay
        write(dir + "broken.ngc", "o<broken> sub\nG0 X0\n");
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(embedded.c_str(), dir.c_str(), r, err));
        CHECK(r.replaced == 0 && r.added == 0 && r.fallback.compare(0, 11, "broken.ngc:") == 0);
        CHECK(r.embedded == files);
        CHECK(m.program().find_sub("extra") < 0 && m.program().find_sub("g28") >= 0);
        remove((dir + "broken.ngc").c_str());
    }
    { // past 64 KB
        write(dir + "huge.ngc", std::string(70000, '\n') + "o<huge> sub\nG0 X1\no<huge> endsub\n");
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(embedded.c_str(), dir.c_str(), r, err));
        CHECK(r.fallback.empty() && r.added == 2); // with extra.ngc from above
        int huge = m.program().find_sub("huge");
        CHECK(huge >= 0 && m.program().subs[huge].line == 70001);
        CHECK(m.program().find_sub("m6") >= 0);
        remove((dir + "huge.ngc").c_str());
    }
    remove((dir + "g28.ngc").c_str()); remove((dir + "extra.ngc").c_str()); remove((dir + "notes.txt").c_str());
    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
