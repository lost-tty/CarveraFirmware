#include "Macros.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <strings.h>

#if defined(__arm__)
#include "DirHandle.h"
#else
#include <dirent.h>
#endif

// the .ngc files' names, without .ngc: the key index_of() looks up
std::vector<std::string> Macros::list(const char *dir)
{
    std::vector<std::string> names;
    if (DIR *d = opendir(dir)) {
        while (struct dirent *e = readdir(d)) {
            std::string name = e->d_name;
            bool ngc = name.size() >= 5 && name.compare(name.size() - 4, 4, ".ngc") == 0;
            if (ngc) names.push_back(name.substr(0, name.size() - 4));
        }
        closedir(d);
    }
    std::sort(names.begin(), names.end());
    return names;
}

void Macros::load(const std::vector<std::string> &search)
{
    dirs = search;
    std::vector<std::string> files;
    for (const std::string &dir : dirs) {
        std::vector<std::string> in = list(dir.c_str());
        files.insert(files.end(), in.begin(), in.end());
    }
    std::sort(files.begin(), files.end(), [](const std::string &a, const std::string &b) {
        return strcasecmp(a.c_str(), b.c_str()) < 0;
    });
    index.clear();
    pool.clear();
    for (unsigned i = 0; i < files.size(); i++) {
        if (i > 0 && strcasecmp(files[i].c_str(), files[i - 1].c_str()) == 0)
            continue;

        index.push_back(pool.size());
        pool += files[i];
        pool += '\0';
    }
    index.shrink_to_fit();
    pool.shrink_to_fit();
}

void Macros::file_changed(const char *path)
{
    for (const std::string &dir : dirs) {
        if (strncmp(path, dir.c_str(), dir.size()) == 0) {
            load(dirs);
            return;
        }
    }
}

int Macros::index_of(const char *sub) const
{
    unsigned lo = 0, hi = index.size();
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        int c = strcasecmp(name(mid), sub);
        if (c == 0)
            return mid;

        if (c < 0) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return -1;
}

bool Macros::has(const char *sub) const
{
    return index_of(sub) >= 0;
}

std::vector<std::string> Macros::paths(const char *sub) const
{
    std::vector<std::string> out;
    int i = index_of(sub);
    if (i < 0)
        return out;

    for (const std::string &dir : dirs) {
        out.push_back(dir + name(i) + ".ngc");
    }
    return out;
}

std::vector<std::string> Macros::names() const
{
    std::vector<std::string> out;
    for (unsigned i = 0; i < index.size(); i++) {
        out.push_back(name(i));
    }
    return out;
}
