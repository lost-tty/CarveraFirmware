// Host test of the script loader: the embedded blob, SD replacements and additions, line mapping, memory use.
// usage: macros_test <blob built by build/macros.sh> <scratch directory>
#include "Macros.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    std::ifstream f(argv[1]); std::stringstream ss; ss << f.rdbuf(); std::string blob = ss.str();
    unsigned files = 0;
    for (size_t p = 0; (p = blob.find("(file: ", p)) != std::string::npos; p += 7) if (p == 0 || blob[p - 1] == '\n') files++;
    std::string dir = std::string(argv[2]) + "/macros/";
    mkdir(dir.c_str(), 0755);

    { // embedded only
        size_t before = live, peak_before = peak = live;
        Macros m; Macros::Report r; std::string err;
        script::Source::opens = 0;
        CHECK(m.load(blob.data(), blob.data() + blob.size(), nullptr, r, err));
        CHECK(err.empty());
        CHECK(script::Source::opens == 0); // the embedded scripts never touch the card
        CHECK(r.embedded == files && r.replaced == 0 && r.added == 0 && r.fallback.empty());
        CHECK(m.program().find_sub("tool_change") >= 0 && m.program().find_sub("atc_change") >= 0);
        int sub = m.program().find_sub("atc_change");
        unsigned at = m.program().controls[sub].offset;
        CHECK(m.file(at) == "atc_change.ngc");                       // the first embedded file, alphabetically
        CHECK(m.source().line_of(at) == 2);                          // the marker line is not part of the segment
        CHECK(m.located("line 2: bad", at) == "atc_change.ngc:2: bad");
        CHECK(m.located("no line", at) == "no line");
        CHECK(m.source().segments.size() == files);
        std::string line; unsigned next;
        CHECK(m.source().line_at(m.source().segments[1].base, line, next)); // segments start at a line and exclude the marker
        CHECK(line.compare(0, 7, "(file: ") != 0);
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
        CHECK(m.load(blob.data(), blob.data() + blob.size(), dir.c_str(), r, err));
        CHECK(r.embedded == files && r.replaced == 1 && r.added == 1 && r.fallback.empty());
        // one size check per file, then one open per switch back to a file segment while validating
        CHECK(script::Source::opens <= 8 * (r.replaced + r.added));
        int extra = m.program().find_sub("extra");
        CHECK(extra >= 0);
        unsigned at = m.program().controls[extra].offset;
        CHECK(m.file(at) == "extra.ngc" && m.source().line_of(at) == 2);
        char buf[32]; snprintf(buf, sizeof(buf), "line %u: x", m.source().line_of(at));
        CHECK(m.located(buf, at) == "extra.ngc:2: x");
        std::string line; unsigned next;                             // the replaced file is read from disk, not from flash
        CHECK(m.source().line_at(m.program().controls[m.program().find_sub("g28")].offset + 11, line, next) && line == "(MSG, custom)");
        m.source().release();
        printf("sd load: %zu bytes live (%zu segments, %zu controls, %zu labels)\n", live - before,
               m.source().segments.size(), m.program().controls.size(), m.program().labels.size());
    }
    { // a broken SD file drops the whole SD set, the embedded scripts stay
        write(dir + "broken.ngc", "o<broken> sub\nG0 X0\n");
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(blob.data(), blob.data() + blob.size(), dir.c_str(), r, err));
        CHECK(r.replaced == 0 && r.added == 0 && r.fallback.compare(0, 11, "broken.ngc:") == 0);
        CHECK(r.embedded == files);
        CHECK(m.program().find_sub("extra") < 0 && m.program().find_sub("g28") >= 0);
        remove((dir + "broken.ngc").c_str());
    }
    { // past the 64 KB the offsets can address
        write(dir + "huge.ngc", std::string(70000, '\n'));
        Macros m; Macros::Report r; std::string err;
        CHECK(m.load(blob.data(), blob.data() + blob.size(), dir.c_str(), r, err));
        CHECK(r.fallback == "scripts are too large" && r.added == 0);
        CHECK(m.program().find_sub("tool_change") >= 0); // the embedded scripts still load
        remove((dir + "huge.ngc").c_str());
    }
    remove((dir + "g28.ngc").c_str()); remove((dir + "extra.ngc").c_str()); remove((dir + "notes.txt").c_str());
    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
