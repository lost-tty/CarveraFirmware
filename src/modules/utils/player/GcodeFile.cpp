#include "GcodeFile.h"
#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include <cstring>

// next executable line; pos advances over everything read, start is where the returned line begins
static bool scan(FILE* fd, char* buf, size_t size, unsigned long& pos, unsigned long& start, unsigned long* long_lines)
{
    bool discard = false;
    while (fgets(buf, size, fd) != NULL) {
        size_t len = strlen(buf);
        start = pos;
        pos += len;
        if (len == 0) continue;
        if (buf[len - 1] != '\n' && !feof(fd)) {
            if (!discard && long_lines) (*long_lines)++;
            discard = true;
            continue;
        }
        if (discard) { discard = false; continue; }
        if (len == 1) continue;
        return true;
    }
    return false;
}

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
    line_count = byte_count = long_lines = pos = 0;
}

bool GcodeFile::next_line(char* buf, size_t size)
{
    unsigned long start;
    if (!scan(fd, buf, size, pos, start, &long_lines)) return false;
    line_count++;
    byte_count += strlen(buf);
    starts[line_count % RECENT] = start;
    return true;
}

void GcodeFile::seek_line(unsigned long n)
{
    char buf[130];
    fseek(fd, 0, SEEK_SET);
    line_count = byte_count = pos = 0;
    while (line_count + 1 < n && next_line(buf, sizeof(buf))) {
        if (line_count % 100 == 0) THEKERNEL->call_event(ON_IDLE);
    }
}

void GcodeFile::list(StreamOutput* stream, unsigned long current, unsigned around)
{
    if (fd == nullptr || line_count == 0) return;
    unsigned long first = current > around ? current - around : 1;
    if (first + RECENT <= line_count) first = line_count - RECENT + 1; // start no longer remembered
    unsigned long saved = pos, p = starts[first % RECENT], start;
    fseek(fd, p, SEEK_SET);
    char buf[130];
    for (unsigned long n = first; n <= current + around && scan(fd, buf, sizeof(buf), p, start, nullptr); n++) {
        buf[strcspn(buf, "\r\n")] = 0;
        stream->printf("%c %5lu  %s\r\n", n == current ? '>' : (n > current && n <= line_count) ? '+' : ' ', n, buf);
    }
    fseek(fd, saved, SEEK_SET);
}
