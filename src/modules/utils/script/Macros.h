#pragma once

#include "Script.h"

#include <string>
#include <vector>

// The machine scripts: the embedded blob (src/macros/*.ngc concatenated, a "(file: name.ngc)" line before each) and,
// per file, a replacement or addition from a directory on the SD card. Nothing is copied into RAM: each file is a
// segment of the source, read from flash or from the card as it runs.
class Macros {
public:
    struct Report { unsigned embedded = 0, replaced = 0, added = 0; std::string fallback; }; // fallback: why the SD files were not used

    bool load(const char *blob, const char *blob_end, const char *dir, Report &report, std::string &err); // dir null: embedded only
    const script::Program &program() const { return prog; }
    script::Source &source() { return src; }
    std::string located(const std::string &err, unsigned offset); // "line N: ..." -> "file.ngc:N: ..."
    std::string file(unsigned offset);                            // the .ngc the offset is in

private:
    struct Embedded { const char *name; uint8_t length; const char *start, *end; };
    static void embedded_files(const char *blob, const char *blob_end, std::vector<Embedded> &files);
    bool build(const char *blob, const char *blob_end, const char *dir, uint32_t replaced,
               const std::vector<std::string> &extra, Report &report, std::string &err);

    script::Source src;
    script::Program prog;
};
