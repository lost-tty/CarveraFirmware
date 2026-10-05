// Host test: c++ -std=c++11 -DFORMAT_HOST_TEST -I../src/libs format_test.cpp ../src/libs/Format.cpp
// Against newlib itself: build for a Cortex-M3 with rdimon.specs and FORMAT_TEST_N small, run in qemu
#include "Format.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

#ifndef FORMAT_TEST_N
#define FORMAT_TEST_N 300000
#endif

static int failures = 0;

template<class... A> static void same(const char *spec, A... args)
{
    char want[512], got[512];
    snprintf(want, sizeof(want), spec, args...);
    fmt::format(got, sizeof(got), spec, args...);
#ifndef __arm__
    // the host's libc signs no nan; newlib-nano, which the firmware has, takes + and space
    if (strstr(want, "nan") && (got[0] == '+' || got[0] == ' ') && strcmp(want, got + 1) == 0)
        return;
#endif
    if (strcmp(want, got) != 0 && failures++ < 20)
        printf("FAIL %s: libc \"%s\", ours \"%s\"\n", spec, want, got);
}

static const char *k_float_specs[] = {
    "%f", "%.0f", "%.2f", "%.3f", "%.4f", "%0.3f", "%0.6f", "%1.18f", "%1.1f", "%1.2f", "%1.3f",
    "%1.4f", "%1.5f", "%10.4f", "%3.1f", "%5.0f", "%5.3f", "%6.2f", "%8.4f", "%g", "%e", "%.3e",
    "%+.2f", "% .1f", "%-10.3f|", "%010.3f", "%#.0f", "%.10g", "%#g", "%G", "%E", "%12.5e", "%.1g",
};

int main()
{
    std::mt19937_64 rng(1);
    for (int i = 0; i < FORMAT_TEST_N; i++) {
        double v;
        switch (i % 6) {
            case 0: v = (double)(float)std::uniform_real_distribution<double>(-1000, 1000)(rng); break;
            case 1: v = (double)(float)std::uniform_real_distribution<double>(-1, 1)(rng); break;
            case 2: { uint64_t b = rng(); memcpy(&v, &b, 8); break; }   // any double
            case 3: v = std::ldexp((double)(rng() >> 11), (int)(rng() % 120) - 100); break;
            case 4: v = std::round(std::uniform_real_distribution<double>(-1e6, 1e6)(rng)) / 8; break;   // ties
            default: v = std::uniform_real_distribution<double>(-1e-3, 1e-3)(rng); break;
        }
        for (const char *spec : k_float_specs) {
            same(spec, v);
        }
    }
    double special[] = {0.0, -0.0, 0.5, 1.5, 2.5, -0.00001, 9.9999999, 99.95, 0.0005, 1e300, -1e-300,
                        5e-324, 1.7976931348623157e308, INFINITY, -INFINITY, NAN, 123456789012.5};
    for (double v : special) {
        for (const char *spec : k_float_specs) {
            same(spec, v);
        }
    }

    const char *int_specs[] = {"%d", "%i", "%5d", "%-5d|", "%05d", "%+d", "% d", "%02d", "%03d", "%.3d", "%0d"};
    for (int i = 0; i < FORMAT_TEST_N / 3; i++) {
        int v = (int)(rng() >> 33) - (1 << 30);
        if (i % 3 == 0)
            v %= 1000;

        for (const char *spec : int_specs) {
            same(spec, v);
        }
        unsigned u = (unsigned)rng();
        same("%u", u);
        same("%x", u);
        same("%X", u);
        same("%02X", u & 0xff);
        same("%08X", u);
        same("%02x", u & 0xff);
        same("%#x", u);
        unsigned long lu = (unsigned long)rng();
        same("%lu", lu);
        same("%02lu", lu % 100);
        same("%5lu", lu % 100000);
        same("%08lx", lu);
        same("%lx", lu);
        same("%ld", (long)lu);
#ifndef __arm__
        same("%llu", (unsigned long long)rng());   // newlib-nano has no ll
        same("%lld", (long long)rng());
#endif
    }
    same("%d %d", INT32_MIN, INT32_MAX);
    same("%c|%3c|%-3c|", 'a', 'b', 'c');
    same("%s|%10s|%-10s|%.3s|%.100s", "abc", "abc", "abc", "abcdef", "abc");
    same("%-20s|%-22s|", "name", "other");
    same("%p|%p", (void *)0x1234, (void *)0x10007a40);
    same("100%%");
    same("%5.1f%%", 42.25);
    same("%*d|%-*d|%.*f", 6, 42, 6, 42, 2, 3.14159);

    char buf[8];
    int n = fmt::format(buf, sizeof(buf), "%s", "0123456789");
    if (n != 10 || strcmp(buf, "0123456") != 0) {
        failures++;
        printf("FAIL truncation: %d \"%s\"\n", n, buf);
    }
    fmt::format(buf, sizeof(buf), "%q");
    if (strcmp(buf, "<?%q>") != 0) {
        failures++;
        printf("FAIL unknown: \"%s\"\n", buf);
    }

    // parsing: the float of the number, as libc reads it
    const char *texts[] = {"0", "-0", "1", "1.", ".5", "-.5", "+2.5", "3.14159", "1e3", "1E-3",
                           "2.5e", "12abc", "   7.25", "0.1", "0.3", "100.000001", "-123.4567",
                           "1e38", "1e-38", "inf", "-Infinity", "nan", ".", "-", "e5", "1.2.3"};
    for (const char *t : texts) {
        char *e1, *e2;
        float a = strtof(t, &e1), b = (float)fmt::parse(t, &e2);
        bool ok = (a == b || (std::isnan(a) && std::isnan(b))) && std::signbit(a) == std::signbit(b) && e1 == e2;
        if (!ok && failures++ < 40)
            printf("FAIL parse \"%s\": libc %.9g +%d, ours %.9g +%d\n", t, a, (int)(e1 - t), b, (int)(e2 - t));
    }
    int off = 0;
    for (int i = 0; i < FORMAT_TEST_N; i++) {
        char t[40];
        double v = std::uniform_real_distribution<double>(-5000, 5000)(rng);
        snprintf(t, sizeof(t), i % 2 ? "%.4f" : "%.7g", v);
        float a = strtof(t, nullptr), b = (float)fmt::parse(t, nullptr);
        if (a != b)
            off++;
    }
    if (off != 0) {
        failures++;
        printf("FAIL parse: %d of %d G-code numbers read differently\n", off, FORMAT_TEST_N);
    }

    printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures != 0;
}
