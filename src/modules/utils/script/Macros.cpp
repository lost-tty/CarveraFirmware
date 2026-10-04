#include "Macros.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#if defined(__arm__)
#include "DirHandle.h"
#else
#include <dirent.h>
#endif

const char Macros::EMBEDDED_DIR[] = "/macros/";

// the .ngc files, by name
std::vector<std::string> Macros::list(const char *dir)
{
    std::vector<std::string> names;
    if (DIR *d = opendir(dir)) {
        while (struct dirent *e = readdir(d)) {
            std::string name = e->d_name;
            bool ngc = name.size() >= 5 && name.compare(name.size() - 4, 4, ".ngc") == 0;
            if (ngc) names.push_back(name);
        }
        closedir(d);
    }
    std::sort(names.begin(), names.end());
    return names;
}

static long file_size(const std::string &path)
{
    FILE *fd = fopen(path.c_str(), "r");
    if (fd == nullptr)
        return -1;

    long n = fseek(fd, 0, SEEK_END) == 0 ? ftell(fd) : -1;
    fclose(fd);
    return n;
}

bool Macros::build(const std::vector<std::string> &paths, std::string &err)
{
    src.clear();
    for (const std::string &path : paths) {
        long size = file_size(path);
        if (size < 0) {
            err = "cannot read " + path;
            return false;
        }

        src.add(path, size);
    }
    return prog.load(src, err);
}

// the SD copy of a file where there is one, the embedded file otherwise, then the SD additions
bool Macros::load(const char *embedded, const char *sd, Report &report, std::string &err)
{
    std::vector<std::string> files = list(embedded), paths;
    report.embedded = files.size();
    report.replaced = report.added = 0;
    for (const std::string &name : files) paths.push_back(embedded + name);

    std::vector<std::string> own = sd != nullptr ? list(sd) : std::vector<std::string>();
    if (!own.empty()) {
        std::vector<std::string> with_sd = paths;
        unsigned replaced = 0, added = 0;
        for (const std::string &name : own) {
            size_t i = std::find(files.begin(), files.end(), name) - files.begin();
            if (i < files.size()) {
                with_sd[i] = sd + name;
                replaced++;
            } else {
                with_sd.push_back(sd + name);
                added++;
            }
        }
        if (build(with_sd, err)) {
            report.replaced = replaced;
            report.added = added;
            return true;
        }

        report.fallback = located(err, prog.error_offset);
    }
    return build(paths, err); // the embedded scripts alone
}

std::string Macros::file(unsigned offset)
{
    int i = src.segment_of(offset);
    return i < 0 ? "" : src.basename(i);
}

std::string Macros::located(const std::string &err, unsigned offset)
{
    unsigned n = 0;
    if (sscanf(err.c_str(), "line %u:", &n) != 1) return err;
    std::string name = file(offset);
    if (name.empty()) return err;
    char buf[16];
    snprintf(buf, sizeof(buf), ":%u:", n);
    return name + buf + err.substr(err.find(':') + 1);
}
