#pragma once

#include "FileSystemLike.h"

// The embedded machine scripts as read-only files under /macros, from the cpio archive
// build/macros.sh packs.
class MacroFS : public mbed::FileSystemLike {
public:
    MacroFS(const char *blob, const char *blob_end);
    mbed::FileHandle *open(const char *filename, int flags) override;
    mbed::DirHandle *opendir(const char *name) override;

private:
    const char *blob, *blob_end;
};
