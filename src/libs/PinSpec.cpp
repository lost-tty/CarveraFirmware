#include "PinSpec.h"
#include <cstdio>
#include <cstdlib>

// Parses the syntax of Pin::from_string(): port.pin followed by modifiers. Malformed input
// yields "nc", and an unknown modifier ends the list.
uint16_t PinSpec::parse(const char *text)
{
    char *end;
    long port = strtol(text, &end, 10);
    if (end == text || port < 0 || port > 4 || *end != '.') return 0;
    const char *p = end + 1;
    long pin = strtol(p, &end, 10);
    if (end == p || pin < 0 || pin > 31) return 0;

    uint16_t s = CONNECTED | port | (pin << PIN_SHIFT);
    for (; *end; end++) {
        switch (*end) {
            case '!': s |= INVERT; break;
            case 'o': s |= OPEN_DRAIN; break;
            case '^': s = (s & ~PULL) | (1 << PULL_SHIFT); break;
            case 'v': s = (s & ~PULL) | (2 << PULL_SHIFT); break;
            case '-': s = (s & ~PULL) | (3 << PULL_SHIFT); break;
            case '@': s |= REPEATER; break;
            case ' ': case '\t': case '\r': case '\n': break;
            default: return s;
        }
    }
    return s;
}

void PinSpec::format(uint16_t s, char *buf, size_t bufsize)
{
    if (!connected(s)) {
        snprintf(buf, bufsize, "nc");
        return;
    }
    static const char pulls[] = { 0, '^', 'v', '-' };
    char mods[5];
    int n = 0;
    if (s & INVERT) mods[n++] = '!';
    if (s & OPEN_DRAIN) mods[n++] = 'o';
    if (pull(s)) mods[n++] = pulls[pull(s)];
    if (s & REPEATER) mods[n++] = '@';
    mods[n] = '\0';
    snprintf(buf, bufsize, "%d.%d%s", port(s), pin(s), mods);
}
