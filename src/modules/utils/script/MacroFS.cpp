#include "MacroFS.h"

#include "DirHandle.h"
#include "FileHandle.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

namespace {

// an entry of the cpio "newc" archive build/macros.sh packs
struct Entry {
    const char *name;
    const char *data;
    size_t size;
};

// a newc header: the magic and thirteen fields of eight hex digits each
struct Header {
    char magic[6];
    char ino[8], mode[8], uid[8], gid[8], nlink[8], mtime[8], filesize[8];
    char devmajor[8], devminor[8], rdevmajor[8], rdevminor[8], namesize[8], check[8];
};
static_assert(sizeof(Header) == 110, "the newc header has no padding");

uint32_t field(const char *p)
{
    char hex[9] = {};
    memcpy(hex, p, 8);
    return strtoul(hex, nullptr, 16);
}

// the name and the data each start on a four-byte boundary
size_t padded(size_t n)
{
    return (n + 3) & ~size_t(3);
}

// false at the trailer
bool next_entry(const char *&p, const char *end, Entry &e)
{
    const Header *h = reinterpret_cast<const Header *>(p);
    if (end - p < ptrdiff_t(sizeof(Header)) || memcmp(h->magic, "070701", 6) != 0)
        return false;

    e.name = p + sizeof(Header);
    e.data = p + padded(sizeof(Header) + field(h->namesize));
    e.size = field(h->filesize);
    if (e.data + e.size > end)
        return false;

    p = e.data + padded(e.size);
    return strcmp(e.name, "TRAILER!!!") != 0;
}

class FlashFile : public mbed::FileHandle {
public:
    FlashFile(const char *start, size_t size) : start(start), size(size) {}

    ssize_t write(const void *, size_t) override { return -1; }

    int close() override
    {
        delete this;
        return 0;
    }

    ssize_t read(void *buffer, size_t length) override
    {
        size_t n = std::min(length, size - at);
        memcpy(buffer, start + at, n);
        at += n;
        return n;
    }

    int isatty() override { return 0; }

    off_t lseek(off_t offset, int whence) override
    {
        off_t to;
        switch (whence) {
            case SEEK_SET: to = offset; break;
            case SEEK_CUR: to = off_t(at) + offset; break;
            case SEEK_END: to = off_t(size) + offset; break;
            default: return -1;
        }
        if (to < 0 || to > off_t(size))
            return -1;

        at = to;
        return to;
    }

    int fsync() override { return 0; }

    off_t flen() override { return size; }

private:
    const char *start;
    size_t size;
    size_t at = 0;
};

class FlashDir : public mbed::DirHandle {
public:
    FlashDir(const char *blob, const char *blob_end) : blob(blob), blob_end(blob_end), at(blob) {}

    int closedir() override
    {
        delete this;
        return 0;
    }

    struct dirent *readdir() override
    {
        Entry f;
        if (!next_entry(at, blob_end, f))
            return nullptr;

        strncpy(entry.d_name, f.name, sizeof(entry.d_name) - 1);
        entry.d_name[sizeof(entry.d_name) - 1] = 0;
        entry.d_fsize = f.size;
        entry.d_isdir = false;
        entry.d_date = entry.d_time = 0;
        return &entry;
    }

    void rewinddir() override { at = blob; }

private:
    const char *blob, *blob_end, *at;
    struct dirent entry;
};

}

MacroFS::MacroFS(const char *blob, const char *blob_end)
    : FileSystemLike(MACROFS_MOUNT), blob(blob), blob_end(blob_end) {}

mbed::FileHandle *MacroFS::open(const char *filename, int flags)
{
    if (flags & (O_WRONLY | O_RDWR))
        return nullptr;

    Entry f;
    for (const char *p = blob; next_entry(p, blob_end, f);) {
        if (strcmp(filename, f.name) == 0)
            return new FlashFile(f.data, f.size);
    }
    return nullptr;
}

mbed::DirHandle *MacroFS::opendir(const char *)
{
    return new FlashDir(blob, blob_end);
}
