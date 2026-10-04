#pragma once

#include "Script.h"

#include <string>
#include <vector>

// The machine scripts: the embedded files under /macros and, per file, a replacement or addition
// from a directory on the SD card. Nothing is copied into RAM: each file is a segment of the
// source, read as it runs.
class Macros {
public:
    struct Report { unsigned embedded = 0, replaced = 0, added = 0; std::string fallback; }; // fallback: why the SD files were not used

    static const char EMBEDDED_DIR[];   // the mount of MacroFS

    // sd null: embedded only
    bool load(const char *embedded, const char *sd, Report &report, std::string &err);
    const script::Program &program() const { return prog; }
    script::Source &source() { return src; }
    std::string located(const std::string &err, unsigned offset); // "line N: ..." -> "file.ngc:N: ..."
    std::string file(unsigned offset);                            // the .ngc the offset is in

private:
    static std::vector<std::string> list(const char *dir);
    bool build(const std::vector<std::string> &paths, std::string &err);

    script::Source src;
    script::Program prog;
};
