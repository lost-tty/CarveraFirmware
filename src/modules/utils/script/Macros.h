#pragma once

#include "Script.h"

#include <cstdint>
#include <string>
#include <vector>

class Macros : public script::Resolver {
public:
    // a file in an earlier directory hides one of the same name in a later one
    void load(const std::vector<std::string> &dirs);
    void file_changed(const char *path);
    bool has(const char *sub) const;
    std::vector<std::string> paths(const char *sub) const override;
    std::vector<std::string> names() const;

private:
    static std::vector<std::string> list(const char *dir);
    int index_of(const char *sub) const;
    const char *name(unsigned i) const { return pool.c_str() + index[i]; }

    std::vector<uint16_t> index;   // into pool, sorted by name
    std::string pool;              // the names without .ngc, each ended by a '\0'
    std::vector<std::string> dirs;
};
