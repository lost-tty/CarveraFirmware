#ifndef _STRINGSTREAM_H_
#define _STRINGSTREAM_H_

#include "StreamOutput.h"

#include <string>

class StringStream : public StreamOutput {
    public:
        StringStream() {}
        int puts(const char *str, int size = 0) { size_t n = size == 0 ? strlen(str) : size; output.append(str, n); return n; }
        void send(uint8_t type, const void *payload, size_t len) { puts((const char *)payload, len); } // captures plain text, not frames
        void clear() { output.clear(); }
        std::string getOutput() const { return output; }

    private:
        std::string output;
};

#endif
