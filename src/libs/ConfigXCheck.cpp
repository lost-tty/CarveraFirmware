// Sim-only check after loading: reports keys registered twice, which find() resolves to
// one struct, and config.txt keys that match a group only with the other separator.
#ifdef CONFIG_XCHECK

#include "ConfigTable.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Names { std::vector<std::string> seen; int dups; };

void collect_name(const char *name, const ConfigTable::Group *, const ConfigTable::Key *,
    void *user)
{
    Names *n = (Names *)user;
    for (const std::string &s : n->seen) {
        if (s == name) {
            printf("config xcheck: DUPLICATE key %s\r\n", name);
            n->dups++;
            return;
        }
    }
    n->seen.push_back(name);
}

// Returns a group whose prefix matches the key's head with the other separator.
const ConfigTable::Group *near_miss(const char *key)
{
    for (const ConfigTable::Group *const *m = config_groups; *m != nullptr; m++) {
        for (const ConfigTable::Group *g = *m; g->prefix != nullptr; g++) {
            size_t plen = strlen(g->prefix);
            if (plen == 0) continue;
            bool flat = g->prefix[plen - 1] == '_';
            size_t blen = flat ? plen - 1 : plen;
            if (strncmp(key, g->prefix, blen) == 0 && key[blen] == (flat ? '.' : '_')) return g;
        }
    }
    return nullptr;
}

int misrouted_keys(const char *path)
{
    FILE *fp = fopen(path, "r");
    if (fp == nullptr) return 0;
    int n = 0;
    char line[132];
    while (fgets(line, sizeof(line), fp) != nullptr) {
        size_t bk = strspn(line, " \t");
        if (line[bk] == '#' || line[bk] == '\0') continue;
        line[bk + strcspn(line + bk, " \t\r\n")] = '\0';
        const ConfigTable::Group *g; const ConfigTable::Key *k;
        if (ConfigTable::find(line + bk, &g, &k)) continue;
        const ConfigTable::Group *near = near_miss(line + bk);
        if (near == nullptr) continue;
        printf("config xcheck: %s misses group %s by its separator\r\n", line + bk, near->prefix);
        n++;
    }
    fclose(fp);
    return n;
}

} // namespace

void config_xcheck_run()
{
    Names names{{}, 0};
    ConfigTable::for_each(collect_name, &names);
    int problems = names.dups + misrouted_keys("/sd/config.txt");
    if (problems == 0) {
        printf("config xcheck: all match\r\n");
    } else {
        printf("config xcheck: %d problem(s)\r\n", problems);
    }
}

#endif // CONFIG_XCHECK
