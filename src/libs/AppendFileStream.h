#ifndef _APPENDFILESTREAM_H_
#define _APPENDFILESTREAM_H_

#include "StreamOutput.h"
#include "string.h"
#include "stdlib.h"

class AppendFileStream : public StreamOutput {
    public:
        AppendFileStream(const char *filename) { fn= strdup(filename); }
        virtual ~AppendFileStream(){ free(fn); }
        int puts(const char*, int size = 0);
        void send(uint8_t type, const void *payload, size_t len) { puts((const char *)payload, len); } // files get plain text, not frames

    private:
        char *fn;
};

#endif
