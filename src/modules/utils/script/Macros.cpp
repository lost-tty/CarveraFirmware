#include "Macros.h"

#include <cstdio>
#include <cstring>

#if defined(__arm__)
#include "DirHandle.h"
#else
#include <dirent.h>
#endif

static const char MARKER[] = "(file: ";
static const unsigned MAX_FILES = 32; // one bit per embedded file in the replaced mask

// the embedded files as (name, content) ranges; the marker line itself is not part of the content
void Macros::embedded_files(const char *p, const char *end, std::vector<Embedded> &files)
{
    while (p < end) {
        const char *nl = (const char *)memchr(p, '\n', end - p);
        if (nl == nullptr) nl = end;
        if (size_t(nl - p) > sizeof(MARKER) && strncmp(p, MARKER, sizeof(MARKER) - 1) == 0) {
            if (!files.empty()) files.back().end = p;
            files.push_back(Embedded{p + sizeof(MARKER) - 1, uint8_t(nl - p - sizeof(MARKER)), nl < end ? nl + 1 : end, end});
        }
        p = nl + 1;
    }
}

static long file_size(const std::string &path)
{
    script::Source::opens++;
    FILE *fd = fopen(path.c_str(), "r");
    if (fd == nullptr) return -1;
    long n = fseek(fd, 0, SEEK_END) == 0 ? ftell(fd) : -1;
    fclose(fd);
    return n;
}

// one segment per file: the SD copy where there is one, the flash text otherwise
bool Macros::build(const char *blob, const char *blob_end, const char *dir, uint32_t replaced,
                   const std::vector<std::string> &extra, Report &report, std::string &err)
{
    std::vector<Embedded> files;
    embedded_files(blob, blob_end, files);
    src.clear();
    report.replaced = report.added = 0;
    for (unsigned i = 0; i < files.size(); i++) {
        const Embedded &f = files[i];
        std::string name(f.name, f.length);
        long size = (replaced & (1u << i)) ? file_size(dir + name) : -1;
        bool ok = size >= 0 ? src.add_file(dir, name, size) : src.add(f.start, f.end - f.start, f.name, f.length);
        if (!ok) { err = "scripts are too large"; return false; }
        if (size >= 0) report.replaced++;
    }
    for (const std::string &name : extra) { // files without an embedded counterpart add subs
        long size = file_size(dir + name);
        if (size < 0) continue;
        if (!src.add_file(dir, name, size)) { err = "scripts are too large"; return false; }
        report.added++;
    }
    return prog.load(src, err);
}

bool Macros::load(const char *blob, const char *blob_end, const char *dir, Report &report, std::string &err)
{
    std::vector<Embedded> files;
    embedded_files(blob, blob_end, files);
    report.embedded = files.size();

    uint32_t replaced = 0;
    std::vector<std::string> extra;
    if (dir != nullptr && files.size() <= MAX_FILES) {
        if (DIR *d = opendir(dir)) {
            while (struct dirent *e = readdir(d)) {
                std::string name = e->d_name;
                if (name.size() < 5 || name.compare(name.size() - 4, 4, ".ngc") != 0) continue;
                unsigned i = 0;
                while (i < files.size() && !(name.size() == files[i].length && name.compare(0, name.size(), files[i].name, files[i].length) == 0)) i++;
                if (i < files.size()) replaced |= 1u << i;
                else extra.push_back(name);
            }
            closedir(d);
        }
    }

    if (replaced != 0 || !extra.empty()) {
        if (build(blob, blob_end, dir, replaced, extra, report, err)) return true;
        report.fallback = located(err, prog.error_offset);
    }
    return build(blob, blob_end, nullptr, 0, std::vector<std::string>(), report, err); // the embedded scripts alone
}

std::string Macros::file(unsigned offset)
{
    int i = src.segment_of(offset);
    return i < 0 ? "" : src.name(i);
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
