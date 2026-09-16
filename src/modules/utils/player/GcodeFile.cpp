#include "GcodeFile.h"
#include "libs/Kernel.h"
#include <cstring>

bool GcodeFile::open(const char* path)
{
    close();
    fd = fopen(path, "r");
    if (fd == nullptr) return false;
    if (fseek(fd, 0, SEEK_END) == 0) {
        file_size = ftell(fd);
        fseek(fd, 0, SEEK_SET);
    }
    return true;
}

void GcodeFile::close()
{
    if (fd != nullptr) fclose(fd);
    fd = nullptr;
    file_size = 0;
    line_count = byte_count = long_lines = 0;
}

bool GcodeFile::next_line(char* buf, size_t size)
{
    bool discard = false;
    while (fgets(buf, size, fd) != NULL) {
        size_t len = strlen(buf);
        if (len == 0) continue;
        if (buf[len - 1] != '\n' && !feof(fd)) {
            if (!discard) long_lines++;
            discard = true;
            continue;
        }
        if (discard) { discard = false; continue; }
        if (len == 1) continue;
        line_count++;
        byte_count += len;
        return true;
    }
    return false;
}

void GcodeFile::seek_line(unsigned long n)
{
    char buf[130];
    fseek(fd, 0, SEEK_SET);
    line_count = byte_count = 0;
    while (line_count + 1 < n && next_line(buf, sizeof(buf))) {
        if (line_count % 100 == 0) THEKERNEL->call_event(ON_IDLE);
    }
}
