#ifndef GCODEFILE_H
#define GCODEFILE_H

#include <cstdio>

class StreamOutput;

// G-code file read one executable line at a time; empty and over-long lines are skipped and not counted
class GcodeFile {
    public:
        bool open(const char* path);
        void close();
        bool is_open() const { return fd != nullptr; }
        bool next_line(char* buf, size_t size);   // false at end of file
        void seek_line(unsigned long n);          // next_line() then returns executable line n (1-based)
        long size() const { return file_size; }
        unsigned long lines() const { return line_count; }   // executable lines returned so far
        unsigned long bytes() const { return byte_count; }
        unsigned long discarded() const { return long_lines; }
        void list(StreamOutput* stream, unsigned long current, unsigned around); // executable lines around `current`, marked > current, + read already
    private:
        static const unsigned RECENT = 64;        // line starts kept, enough for the planner queue plus the listing
        FILE* fd = nullptr;
        long file_size = 0;
        unsigned long line_count = 0, byte_count = 0, long_lines = 0;
        unsigned long pos = 0;                    // file offset after the last fgets
        unsigned long starts[RECENT];             // offset of executable line n at n % RECENT
};

#endif
